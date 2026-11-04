// v7.2: bounded, deduplicated recovery. Header/codec/wire interfaces unchanged.
// The legacy Options::recovery_budget_ms field is now an ENTIRE decoder-call
// budget when positive (0 still disables it), including dot detection/fast reuse.
// It is COOPERATIVE: checked inside long loops, not a hardware deadline.
// All coordinates still require current-image geometry + exact 8x8 verification.
// Reference considered: Hostettler et al., ICRA 2016, II-B (bounded refinement).
// This is NOT a port of libdots and does not import its probabilistic bit repair.
#include "anoto_engine.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <cstring>
#include <cstdio>
namespace anoto {
    namespace {
        constexpr double pi=3.14159265358979323846;
        using Clock=std::chrono::steady_clock;
        double ms(Clock::time_point t) {
            return std::chrono::duration<double,std::milli>(Clock::now()-t).count();
        }
        struct FrameBudgetExceeded { const char* stage; };
        struct WorkBudget {
            bool enabled=false;
            Clock::time_point deadline{};
            void start(Clock::time_point begin,double milliseconds) {
                enabled=milliseconds>0;
                if(enabled)deadline=begin+std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double,std::milli>(milliseconds));
            }
            void check(const char* stage) const {
                if(enabled && Clock::now()>=deadline)throw FrameBudgetExceeded{stage};
            }
        };
        struct FinishBudgetPause {
            WorkBudget& budget;bool previous;
            explicit FinishBudgetPause(WorkBudget& b):budget(b),previous(b.enabled){budget.enabled=false;}
            ~FinishBudgetPause(){budget.enabled=previous;}
        };
        // Exclusive, exception-safe stage accounting. A nested address decode
        // pauses geometry accounting; an interrupted fit is still timed.
        struct StageAccounting {
            double* active=nullptr;
            Clock::time_point stamp{};
            void switch_to(double* next) {
                const auto now=Clock::now();
                if(active)*active+=std::chrono::duration<double,std::milli>(now-stamp).count();
                active=next;stamp=now;
            }
        };
        struct StageScope {
            StageAccounting& accounting;
            double* parent;
            StageScope(StageAccounting& a,double& value):accounting(a),parent(a.active) {
                accounting.switch_to(&value);
            }
            ~StageScope(){accounting.switch_to(parent);}
            StageScope(const StageScope&)=delete;
            StageScope& operator=(const StageScope&)=delete;
        };
        double sq(double v) {
            return v*v;
        }
        Point operator+(Point a,Point b) {
            return {
                a.x+b.x,a.y+b.y
            };
        }
        Point operator-(Point a,Point b) {
            return {
                a.x-b.x,a.y-b.y
            };
        }
        Point operator*(Point a,double s) {
            return {
                a.x*s,a.y*s
            };
        }
        double norm2(Point p) {
            return p.x*p.x+p.y*p.y;
        }
        double norm(Point p) {
            return std::sqrt(norm2(p));
        }
        double med(std::vector<double> v) {
            if(v.empty())return 0;
            size_t k=v.size()/2;
            std::nth_element(v.begin(),v.begin()+k,v.end());
            double x=v[k];
            if(v.size()%2==0)x=(x+*std::max_element(v.begin(),v.begin()+k))*.5;
            return x;
        }
        struct Mat {
            double a[9]= {
                1,0,0,0,1,0,0,0,1
            };
            Point apply(Point p) const {
                double d=a[6]*p.x+a[7]*p.y+a[8];
                if(std::abs(d)<1e-10)throw std::runtime_error("PROJECTIVE_HORIZON");
                const double invd=1./d;
                return {
                    (a[0]*p.x+a[1]*p.y+a[2])*invd,(a[3]*p.x+a[4]*p.y+a[5])*invd
                };
            }
            bool inverse(Mat& b) const {
                const double A=a[4]*a[8]-a[5]*a[7],B=a[5]*a[6]-a[3]*a[8],C=a[3]*a[7]-a[4]*a[6];
                double d=a[0]*A+a[1]*B+a[2]*C;
                if(!std::isfinite(d)||std::abs(d)<1e-14)return false;
                const double invd=1./d;
                b= {
                    {
                        A*invd,(a[2]*a[7]-a[1]*a[8])*invd,(a[1]*a[5]-a[2]*a[4])*invd,
                        B*invd,(a[0]*a[8]-a[2]*a[6])*invd,(a[2]*a[3]-a[0]*a[5])*invd,
                        C*invd,(a[1]*a[6]-a[0]*a[7])*invd,(a[0]*a[4]-a[1]*a[3])*invd
                    }
                };
                return true;
            }
        };
        Mat mul(const Mat& a,const Mat& b) {
            Mat c;
            for(int i=0;i<3;++i)for(int j=0;j<3;++j) {
                c.a[3*i+j]=0;
                for(int k=0;k<3;++k)c.a[3*i+j]+=a.a[3*i+k]*b.a[3*k+j];
            }
            return c;
        }
        Point derivative(const Mat& m,Point p,Point direction) {
            Point q=m.apply(p);
            double d=m.a[6]*p.x+m.a[7]*p.y+m.a[8];
            return {
                ((m.a[0]-q.x*m.a[6])*direction.x+(m.a[1]-q.x*m.a[7])*direction.y)/d,((m.a[3]-q.y*m.a[6])*direction.x+(m.a[4]-q.y*m.a[7])*direction.y)/d
            };
        }
        // Normalized least-squares homography. Only an 8x9 system, not a large SVD.
        bool fitH(const std::vector<Point>& src,const std::vector<Point>& dst,Mat& out,const std::vector<double>* weights=nullptr,const WorkBudget* budget=nullptr) {
            if(budget)budget->check("solve");
            if(src.size()!=dst.size()||src.size()<8)return false;
            Point a {
            },b {
            };
            double mass=0;
            for(size_t i=0;i<src.size();++i) {
                if(budget && (i&15u)==0)budget->check("solve");
                double w=weights?(*weights)[i]:1.;
                a=a+src[i]*w;
                b=b+dst[i]*w;
                mass+=w;
            }
            if(mass<5)return false;
            a=a*(1/mass);
            b=b*(1/mass);
            double va=0,vb=0;
            for(size_t i=0;i<src.size();++i) {
                if(budget && (i&15u)==0)budget->check("solve");
                double w=weights?(*weights)[i]:1.;
                va+=w*norm2(src[i]-a);
                vb+=w*norm2(dst[i]-b);
            }
            if(va<1e-12||vb<1e-12)return false;
            double sa=std::sqrt(2*mass/va),sb=std::sqrt(2*mass/vb);
            double n[8][9] {
            };
            for(size_t i=0;i<src.size();++i) {
                if(budget && (i&15u)==0)budget->check("solve");
                double w=weights?(*weights)[i]:1.;
                if(w<1e-12)continue;
                Point q=(src[i]-a)*sa,p=(dst[i]-b)*sb;
                // v7: a homography row has only five nonzero coefficients.
                // Accumulate those terms rather than multiplying the structural
                // zeros. The least-squares model and weights are unchanged.
                const int ix[2][5]={{0,1,2,6,7},{3,4,5,6,7}};
                const double v[2][5]={{q.x,q.y,1.,-p.x*q.x,-p.x*q.y},
                                      {q.x,q.y,1.,-p.y*q.x,-p.y*q.y}};
                const double rhs[2]={p.x,p.y};
                for(int r=0;r<2;++r)for(int j=0;j<5;++j){
                    const double wv=weights?w*v[r][j]:v[r][j];
                    for(int k=j;k<5;++k)n[ix[r][j]][ix[r][k]]+=wv*v[r][k];
                    n[ix[r][j]][8]+=wv*rhs[r];
                }
            }
            for(int j=0;j<8;++j)for(int k=0;k<j;++k)n[j][k]=n[k][j];
            for(int j=0;j<8;++j) {
                if(budget)budget->check("solve");
                int pivot=j;
                for(int k=j+1;k<8;++k)if(std::abs(n[k][j])>std::abs(n[pivot][j]))pivot=k;
                if(std::abs(n[pivot][j])<1e-10)return false;
                if(pivot!=j)for(int k=j;k<9;++k)std::swap(n[pivot][k],n[j][k]);
                const double invpivot=1./n[j][j];
                for(int k=j;k<9;++k)n[j][k]*=invpivot;
                for(int i=0;i<8;++i)if(i!=j) {
                    double f=n[i][j];
                    for(int k=j;k<9;++k)n[i][k]-=f*n[j][k];
                }
            }
            Mat h;
            for(int i=0;i<8;++i)h.a[i]=n[i][8];
            h.a[8]=1;
            Mat ta= {
                {
                    sa,0,-sa*a.x,0,sa,-sa*a.y,0,0,1
                }
            },ib= {
                {
                    1/sb,0,b.x,0,1/sb,b.y,0,0,1
                }
            };
            out=mul(mul(ib,h),ta);
            double s=out.a[8];
            if(std::abs(s)<1e-12)return false;
            const double invs=1./s;
            for(auto& v:out.a) {
                v*=invs;
                if(!std::isfinite(v))return false;
            }
            return true;
        }
        // Two 2x2 covariance solves, shared inverse. All coordinates are local
        // (a few grid cells), never absolute 10 m floor coordinates.
        bool fitAffine(const std::vector<Point>& src,const std::vector<Point>& dst,Mat& out,const WorkBudget* budget=nullptr){
            if(budget)budget->check("solve");
            if(src.size()!=dst.size()||src.size()<8)return false;
            Point a{},b{};
            for(size_t i=0;i<src.size();++i){if(budget && (i&15u)==0)budget->check("solve");a=a+src[i];b=b+dst[i];}
            const double invn=1./src.size();a=a*invn;b=b*invn;
            double xx=0,xy=0,yy=0,ux=0,uy=0,vx=0,vy=0;
            for(size_t i=0;i<src.size();++i){if(budget && (i&15u)==0)budget->check("solve");
                Point q=src[i]-a,p=dst[i]-b;
                xx+=q.x*q.x;xy+=q.x*q.y;yy+=q.y*q.y;
                ux+=p.x*q.x;uy+=p.x*q.y;vx+=p.y*q.x;vy+=p.y*q.y;
            }
            const double det=xx*yy-xy*xy;
            if(!std::isfinite(det)||det<1e-9||det<1e-7*(xx+yy)*(xx+yy))return false;
            const double inv=1./det;
            const double aa=(ux*yy-uy*xy)*inv,bb=(uy*xx-ux*xy)*inv;
            const double cc=(vx*yy-vy*xy)*inv,dd=(vy*xx-vx*xy)*inv;
            // Reject reflections, collapse and wild distortion of the last profile.
            const double area=aa*dd-bb*cc;
            if(!std::isfinite(area)||area<.50||area>2.||
               aa*aa+bb*bb+cc*cc+dd*dd>5.)return false;
            out=Mat{{aa,bb,b.x-aa*a.x-bb*a.y,cc,dd,b.y-cc*a.x-dd*a.y,0,0,1}};
            return true;
        }
        // v3: EXACT rational form of the old background-plane centroid.
        // L/R/T/B are integer edge sums; D=2*r+1. Multiplying all weights by
        // S=4*r*D eliminates every per-pixel floating-point op and division:
        // w_num = r*(L+R+T+B)+2*(R-L)*x+2*(B-T)*y - S*(pixel+2).
        // The common positive S cancels in sum(x*w)/sum(w). Not binary-only,
        // no downsampling, no approximated Q8/Q16 background; clipping unchanged.
        template<class Acc> bool integer_centroid_sum(GrayView im,int cx,int cy,int r,
                                                       int base,int dx,int dy,int scale,
                                                       Point& refined) {
            Acc mass=0,mx=0,my=0;
            int row_bg=base-dy*r-dx*r;
            for(int y=-r;y<=r;++y,row_bg+=dy){
                const auto* row=im.data+size_t(cy+y)*im.stride+cx-r;
                int bg=row_bg;
                for(int x=-r;x<=r;++x,bg+=dx,++row){
                    const int w=std::max(0,bg-scale*(int(*row)+2));
                    mass+=w;mx+=Acc(x)*w;my+=Acc(y)*w;
                }
            }
            if(mass<=Acc(20)*scale)return false;
            const double inv=1./double(mass);
            refined={cx+double(mx)*inv,cy+double(my)*inv};
            return std::isfinite(refined.x)&&std::isfinite(refined.y);
        }
        bool integer_centroid(GrayView im,int cx,int cy,int r,Point& refined){
            if(r<1||r>64||cx-r<0||cy-r<0||cx+r>=im.width||cy+r>=im.height)return false;
            int l=0,rr=0,t=0,b=0;
            for(int u=-r;u<=r;++u){
                l+=im.data[size_t(cy+u)*im.stride+cx-r];rr+=im.data[size_t(cy+u)*im.stride+cx+r];
                t+=im.data[size_t(cy-r)*im.stride+cx+u];b+=im.data[size_t(cy+r)*im.stride+cx+u];
            }
            const int scale=4*r*(2*r+1),base=r*(l+rr+t+b);
            // Conservative bound for r<=10: |moment| <= r*D^2*510*scale
            // <= 1,889,244,000 < INT32_MAX. Larger support uses 64-bit moments.
            if(r<=10)return integer_centroid_sum<std::int32_t>(im,cx,cy,r,base,2*(rr-l),2*(b-t),scale,refined);
            return integer_centroid_sum<std::int64_t>(im,cx,cy,r,base,2*(rr-l),2*(b-t),scale,refined);
        }
        struct Dot {
            Point p;
            int area=0,x0=0,y0=0,x1=0,y1=0;
            double pitch=0;
        };
        struct Knn {
            int id[9];
            double d[9];
            int k=0;
        };
        // Balanced 2-D kd-tree; avoids quadratic all-pairs distance arrays.
        class KD {
            const std::vector<Dot>* p_=nullptr;
            std::vector<int> order_;
            void build(int l,int r,int axis) {
                if(l>=r)return;
                int m=(l+r)/2;
                std::nth_element(order_.begin()+l,order_.begin()+m,order_.begin()+r,[&](int a,int b) {
                    return axis?(*p_)[a].p.y<(*p_)[b].p.y:(*p_)[a].p.x<(*p_)[b].p.x;
                });
                build(l,m,1-axis);
                build(m+1,r,1-axis);
            }
            void visit(int l,int r,int axis,Point p,int exclude,Knn& k)const {
                if(l>=r)return;
                int m=(l+r)/2,id=order_[m];
                Point d=(*p_)[id].p-p;
                double ds=norm2(d);
                if(id!=exclude&&ds<k.d[k.k-1]) {
                    int pos=k.k-1;
                    while(pos>0&&ds<k.d[pos-1]) {
                        k.d[pos]=k.d[pos-1];
                        k.id[pos]=k.id[pos-1];
                        --pos;
                    }
                    k.d[pos]=ds;
                    k.id[pos]=id;
                }
                double delta=axis?d.y:d.x;
                if(delta>0) {
                    visit(l,m,1-axis,p,exclude,k);
                    if(delta*delta<k.d[k.k-1])visit(m+1,r,1-axis,p,exclude,k);
                }
                else {
                    visit(m+1,r,1-axis,p,exclude,k);
                    if(delta*delta<k.d[k.k-1])visit(l,m,1-axis,p,exclude,k);
                }
            }
            public:
            void set(const std::vector<Dot>& p) {
                p_=&p;
                order_.resize(p.size());
                std::iota(order_.begin(),order_.end(),0);
                build(0,int(p.size()),0);
            }
            Knn query(Point p,int exclude=-1,int count=8)const {
                Knn k;
                k.k=std::min(count,int(p_->size())-(exclude>=0));
                for(int j=0;j<9;++j) {
                    k.id[j]=-1;
                    k.d[j]=1e300;
                }
                if(k.k>0)visit(0,int(p_->size()),0,p,exclude,k);
                return k;
            }
            size_t bytes()const {
                return order_.capacity()*sizeof(int);
            }
        };
        struct Run {
            int x0,x1,label;
        };
        struct Component {
            int parent=0,area=0,x0=0,x1=0,y0=0,y1=0;
            std::int64_t sumx=0,sumy=0;
        };
        int root(std::vector<Component>& c,int i) {
            int r=i;
            while(c[r].parent!=r)r=c[r].parent;
            while(c[i].parent!=i) {
                int j=c[i].parent;
                c[i].parent=r;
                i=j;
            }
            return r;
        }
        void join(std::vector<Component>& c,int i,int j) {
            i=root(c,i);
            j=root(c,j);
            if(i==j)return;
            if(c[i].area<c[j].area)std::swap(i,j);
            auto& a=c[i];
            auto& b=c[j];
            b.parent=i;
            a.area+=b.area;
            a.sumx+=b.sumx;
            a.sumy+=b.sumy;
            a.x0=std::min(a.x0,b.x0);
            a.x1=std::max(a.x1,b.x1);
            a.y0=std::min(a.y0,b.y0);
            a.y1=std::max(a.y1,b.y1);
        }
        int otsu(GrayView im) {
            std::uint64_t hist[256] {
            };
            for(int y=0;y<im.height;++y) {
                auto p=im.data+size_t(y)*im.stride;
                for(int x=0;x<im.width;++x)++hist[p[x]];
            }
            std::uint64_t total=std::uint64_t(im.width)*im.height,n=0,s=0,all=0;
            for(int i=0;i<256;++i)all+=i*hist[i];
            double best=-1;
            int t=0;
            for(int i=0;i<255;++i) {
                n+=hist[i];
                s+=i*hist[i];
                if(!n||n==total)continue;
                double d=double(s)/n-double(all-s)/(total-n),score=double(n)*(total-n)*d*d;
                if(score>best) {
                    best=score;
                    t=i;
                }
            }
            return t;
        }
        Point offset(int s,double r) {
            switch(s) {
                case 0:return {
                    0,-r
                };
                case 1:return {
                    r,0
                };
                case 2:return {
                    -r,0
                };
                default:return {
                    0,r
                };
            }
        }
        int classify(Point residual,double ratio,double& error,double& margin) {
            double best=1e9,second=1e9;
            int symbol=0;
            for(int s=0;s<4;++s) {
                double d=norm2(residual-offset(s,ratio));
                if(d<best) {
                    second=best;
                    best=d;
                    symbol=s;
                }
                else if(d<second)second=d;
            }
            error=std::sqrt(best);
            margin=std::sqrt(second)-error;
            return symbol;
        }
        int classify_squared(Point d,double ratio,double& error2){
            const double projection[4]={-d.y,d.x,-d.x,d.y};
            int symbol=0;
            for(int k=1;k<4;++k)if(projection[k]>projection[symbol])symbol=k;
            error2=norm2(d-offset(symbol,ratio));
            return symbol;
        }
        struct Basis {
            Point x,y;
            double pitch=0;
            Point inv(Point p)const {
                double d=x.x*y.y-x.y*y.x;
                return {
                    (y.y*p.x-y.x*p.y)/d,(-x.y*p.x+x.x*p.y)/d
                };
            }
        };
        struct Union {
            int parent,size;
            int x,y;
        };
        int ufroot(std::vector<Union>& u,int i) {
            if(u[i].parent!=i) {
                int p=u[i].parent,r=ufroot(u,p);
                u[i].x+=u[p].x;
                u[i].y+=u[p].y;
                u[i].parent=r;
            }
            return u[i].parent;
        }
        struct Edge {
            float error;
            int a,b,dx,dy;
        };
        struct Cell {
            int id=-1;
            std::uint8_t symbol=255;
            float error=0;
        };
        struct Grid {
            Mat h;
            int w=0,height=0;
            std::vector<Cell> cells;
            std::vector<Point> points;
            std::vector<std::uint8_t> symbols;
            double median_error=0;
        };
        struct FitAnswer {
            Grid grid;
            Address addr;
            int block_r=0,block_c=0,blocks=0;
            int rx=1,ry=0,sx=0,sy=1,bx=0,by=0;
        };
    }
    // local
    struct Engine::Impl {
        Options o;
        std::vector<Dot> dots,selected;
        std::vector<Run> prev,curr;
        std::vector<Component> comps,compact;
        std::vector<int> mark,remap,columns;
        std::vector<Dot> filtered;
        KD kd;
        Mat previous;
        bool has_previous=false;
        int preferred=0;
        std::size_t workspace_peak=0;
        CameraProfile profile;
        Mat cal,cal_inv,tracked,older_tracked;
        bool has_track=false;
        int history_count=0,last_threshold=127;
        double prediction_scale=1.;
        bool cached_scan_ready=false;
        Point anchor{},last_camera{},older_camera{};
        double last_heading=0,older_heading=0;
        bool gate_appearance=false;
        std::vector<Point> fit_src,fit_dst,rectified;
        std::vector<int> fit_associations,previous_associations;
        int recovery_count=0;
        Clock::time_point processing_start;
        WorkBudget budget;
        StageAccounting accounting;
        std::vector<Point> cold_key,last_cold_key;
        unsigned cold_variants_done=0;
        const char* last_rejection="NONE";
        int decode_calls=0,duplicate_candidates=0;
        bool reject(const char* why){last_rejection=why;return false;}
        void checkpoint(const char* stage) const {budget.check(stage);}
        std::string failure_reason(const char* kind,const char* stage=nullptr) const {
            char text[96];
            std::snprintf(text,sizeof(text),"%s%s%s;last=%s;dec=%d;dup=%d",
                kind,stage?"@":"",stage?stage:"",last_rejection,decode_calls,duplicate_candidates);
            return text;
        }
        bool recovery_allowed() const {
            checkpoint("recovery");
            return !o.fixed_camera || recovery_count<o.max_recovery_fits;
        }

        explicit Impl(const Options& opt):o(opt) {
            if(!(o.pitch_mm>0&&o.offset_mm/o.pitch_mm>.05&&o.offset_mm/o.pitch_mm<.25))throw std::invalid_argument("Invalid print pitch/offset");
            if(o.max_dots<128||o.max_dots>16000||o.max_fit_points<80||o.max_fit_points>1500||o.min_confirm<1||o.max_blocks<o.min_confirm)throw std::invalid_argument("Invalid limits");
            if(!std::isfinite(o.floor_width_mm)||!std::isfinite(o.floor_height_mm)||o.floor_width_mm/o.pitch_mm<8||o.floor_height_mm/o.pitch_mm<8||o.floor_width_mm/o.pitch_mm>Codec::coordinate_period()-1||o.floor_height_mm/o.pitch_mm>Codec::coordinate_period()-1)throw std::invalid_argument("Invalid floor extent");
            if(o.section_x < -1 || o.section_x>62 || o.section_y < -1 || o.section_y>62)throw std::invalid_argument("section must be -1 or 0..62");
            if(o.patch_side<10||o.patch_side>32||o.max_recovery_fits<1||
               !std::isfinite(o.max_predict_mm)||o.max_predict_mm<=0||
               !std::isfinite(o.max_predict_deg)||o.max_predict_deg<=0||
               !std::isfinite(o.recovery_budget_ms)||o.recovery_budget_ms<0||o.recovery_budget_ms>3600000.||
               !std::isfinite(o.camera_forward_mm)||!std::isfinite(o.camera_right_mm))
                throw std::invalid_argument("Invalid fixed-camera configuration");
            if(o.target_confirm<o.min_confirm||o.target_confirm>o.max_blocks)
                throw std::invalid_argument("target_confirm must be >= min_confirm and <= max_blocks");
            fit_associations.reserve(o.max_fit_points*4);previous_associations.reserve(o.max_fit_points*4);
            fit_src.reserve(o.max_fit_points);fit_dst.reserve(o.max_fit_points);rectified.reserve(o.max_fit_points);
            dots.reserve(o.max_dots);
            filtered.reserve(o.max_dots);
            selected.reserve(o.max_fit_points);
            cold_key.reserve(o.max_fit_points);last_cold_key.reserve(o.max_fit_points);
        }
        std::size_t bytes()const {
            return sizeof(*this)+(dots.capacity()+selected.capacity()+filtered.capacity())*sizeof(Dot)
            +(prev.capacity()+curr.capacity())*sizeof(Run)+(comps.capacity()+compact.capacity())*sizeof(Component)
            +(mark.capacity()+remap.capacity()+columns.capacity())*sizeof(int)+kd.bytes()
            +(fit_src.capacity()+fit_dst.capacity()+rectified.capacity())*sizeof(Point)
            +(fit_associations.capacity()+previous_associations.capacity())*sizeof(int)
            +(cold_key.capacity()+last_cold_key.capacity())*sizeof(Point);
        }
        void emit(const Component& c,GrayView im,Point ref,Observation& obs) {
            int bw=c.x1-c.x0+1,bh=c.y1-c.y0+1;
            double aspect=double(bw)/bh,fill=double(c.area)/(bw*bh);
            double limit=.02*sq(std::min(im.width,im.height));
            if(c.area<2||c.area>limit||aspect<.22||aspect>4.5||fill<.22||c.x0<=0||c.y0<=0||c.x1>=im.width-1||c.y1>=im.height-1)return;
            if(gate_appearance && profile.valid) {
                if(c.area<profile.area_px*.28||c.area>profile.area_px*3.5)return;
            }
            Dot d;
            d.p= {
                double(c.sumx)/c.area,double(c.sumy)/c.area
            };
            d.area=c.area;
            d.x0=c.x0;
            d.y0=c.y0;
            d.x1=c.x1;
            d.y1=c.y1;
            auto farther=[&](const Dot& a,const Dot& b) {
                return norm2(a.p-ref)<norm2(b.p-ref);
            };
            if(int(dots.size())<o.max_dots) {
                dots.push_back(d);
                std::push_heap(dots.begin(),dots.end(),farther);
            }
            else {
                ++obs.dropped_dots;
                if(norm2(d.p-ref)<norm2(dots.front().p-ref)) {
                    std::pop_heap(dots.begin(),dots.end(),farther);
                    dots.back()=d;
                    std::push_heap(dots.begin(),dots.end(),farther);
                }
            }
        }
        void detect(GrayView im,int mode,Point ref,Observation& obs,bool reuse=false) {
            checkpoint("dots");
            StageScope stage_timer(accounting,obs.time.dots_ms);
            checkpoint("dots");
            last_cold_key.clear();cold_variants_done=0;
            dots.clear();
            prev.clear();
            comps.clear();
            gate_appearance=reuse&&profile.valid;
            int threshold=mode==0?(gate_appearance?profile.threshold:otsu(im)):0;
            if(!gate_appearance)last_threshold=mode==0?threshold:otsu(im);
            int radius=std::max(10,std::min(150,int(std::min(im.width,im.height)*.04*(mode==2?2.2:1))));
            if(gate_appearance)radius=std::clamp(int(std::ceil(profile.diameter_px*3)),7,150);
            if(mode) {
                columns.assign(im.width,0);
                for(int y=0;y<=std::min(im.height-1,radius);++y)for(int x=0;x<im.width;++x)columns[x]+=im.data[size_t(y)*im.stride+x];
            }
            for(int y=0;y<im.height;++y) {
                if((y&7)==0)checkpoint("dots");
                if(mode&&y>0) {
                    int add=y+radius,sub=y-radius-1;
                    for(int x=0;x<im.width;++x) {
                        if(add<im.height)columns[x]+=im.data[size_t(add)*im.stride+x];
                        if(sub>=0)columns[x]-=im.data[size_t(sub)*im.stride+x];
                    }
                }
                curr.clear();
                auto row=im.data+size_t(y)*im.stride;
                int horizontal=0;
                if(mode)for(int x=0;x<=std::min(im.width-1,radius);++x)horizontal+=columns[x];
                int run_start=-1;
                for(int x=0;x<=im.width;++x) {
                    bool black=false;
                    if(x<im.width) {
                        if(mode) {
                            if(x>0) {
                                int add=x+radius,sub=x-radius-1;
                                if(add<im.width)horizontal+=columns[add];
                                if(sub>=0)horizontal-=columns[sub];
                            }
                            int nx=std::min(im.width-1,x+radius)-std::max(0,x-radius)+1,ny=std::min(im.height-1,y+radius)-std::max(0,y-radius)+1;
                            black=int(row[x])*nx*ny < horizontal-5*nx*ny;
                        }
                        else black=row[x]<=threshold;
                    }
                    if(black&&run_start<0)run_start=x;
                    if(!black&&run_start>=0) {
                        int id=int(comps.size()),length=x-run_start;
                        Component c;
                        c.parent=id;
                        c.area=length;
                        c.x0=run_start;
                        c.x1=x-1;
                        c.y0=c.y1=y;
                        c.sumx=std::int64_t(run_start+x-1)*length/2;
                        c.sumy=std::int64_t(y)*length;
                        comps.push_back(c);
                        curr.push_back( {
                            run_start,x-1,id
                        });
                        run_start=-1;
                    }
                }
                size_t j=0;
                for(auto& run:curr) {
                    while(j<prev.size()&&prev[j].x1<run.x0-1)++j;
                    for(size_t k=j;k<prev.size()&&prev[k].x0<=run.x1+1;++k)join(comps,run.label,prev[k].label);
                }
                mark.assign(comps.size(),0);
                for(auto& r:curr) {
                    r.label=root(comps,r.label);
                    mark[r.label]=1;
                }
                for(size_t i=0;i<comps.size();++i)if(comps[i].parent==int(i)&&!mark[i])emit(comps[i],im,ref,obs);
                remap.assign(comps.size(),-1);
                compact.clear();
                for(auto& r:curr) {
                    if(remap[r.label]<0) {
                        Component c=comps[r.label];
                        int id=int(compact.size());
                        remap[r.label]=id;
                        c.parent=id;
                        compact.push_back(c);
                    }
                    r.label=remap[r.label];
                }
                comps.swap(compact);
                prev.swap(curr);
            }
            for(const auto& c:comps)emit(c,im,ref,obs);
            if(dots.size()<8)return;
            if(!gate_appearance)kd.set(dots);
            filtered.clear();
            for(int i=0;i<int(dots.size());++i) {
                if((i&15)==0)checkpoint("dots");
                double p=0,nearest=0;
                if(gate_appearance) {
                    Point q=cal_inv.apply(dots[i].p);
                    double px=norm(derivative(cal,q,{1,0})),py=norm(derivative(cal,q,{0,1}));
                    p=.5*(px+py);nearest=std::min(px,py)*.6;
                } else {
                    auto k=kd.query(dots[i].p,i,6);
                    if(k.k<4)continue;
                    p=(std::sqrt(k.d[1])+std::sqrt(k.d[2]))*.5;
                    nearest=std::sqrt(k.d[0]);
                }
                double rel=dots[i].area/std::max(1.,p*p);
                if(p<3.5||rel<.0006||rel>.55)continue;
                Dot d=dots[i];
                d.pitch=p;
                int radius=std::min(55,std::max(2,std::min(int(std::ceil(std::max(d.x1-d.x0+1,d.y1-d.y0+1)*.5))+2,int(.42*nearest))));
                int cx=int(std::lround(d.p.x)),cy=int(std::lround(d.p.y));
                if(cx-radius>=0&&cy-radius>=0&&cx+radius<im.width&&cy+radius<im.height) {
                    if(o.integer_centroids){
                        Point refined;
                        if(integer_centroid(im,cx,cy,radius,refined)&&
                           norm2(refined-d.p)<sq(std::max(1.2,std::max(d.x1-d.x0+1,d.y1-d.y0+1)*.25)))d.p=refined;
                    }else{
                    double left=0,right=0,top=0,bottom=0;
                    for(int u=-radius;u<=radius;++u) {
                        left+=im.data[size_t(cy+u)*im.stride+cx-radius];
                        right+=im.data[size_t(cy+u)*im.stride+cx+radius];
                        top+=im.data[size_t(cy-radius)*im.stride+cx+u];
                        bottom+=im.data[size_t(cy+radius)*im.stride+cx+u];
                    }
                    double denom=2*radius+1;
                    left/=denom;
                    right/=denom;
                    top/=denom;
                    bottom/=denom;
                    double base=(left+right+top+bottom)*.25,m=0,xx=0,yy=0;
                    for(int v=-radius;v<=radius;++v)for(int u=-radius;u<=radius;++u) {
                        double bg=base+(right-left)*u/(2*radius)+(bottom-top)*v/(2*radius);
                        double w=std::max(0.,bg-im.data[size_t(cy+v)*im.stride+cx+u]-2);
                        m+=w;
                        xx+=u*w;
                        yy+=v*w;
                    }
                    if(m>20) {
                        Point refined= {
                            cx+xx/m,cy+yy/m
                        };
                        if(norm(refined-d.p)<std::max(1.2,std::max(d.x1-d.x0+1,d.y1-d.y0+1)*.25))d.p=refined;
                    }
                    } // reference floating-point centroid
                }
                filtered.push_back(d);
            }
            dots.swap(filtered);
        }
        bool initial(const std::vector<Dot>& p,Basis& b,std::vector<Knn>& neighbours,double min_pitch=3.5) {
            checkpoint("initial");
            if(p.size()<64)return reject("INITIAL_GEOMETRY");
            KD tree;
            tree.set(p);
            neighbours.resize(p.size());
            std::vector<double> lengths;
            std::vector<Point> vectors;
            lengths.reserve(p.size()*4);
            vectors.reserve(p.size()*4);
            for(int i=0;i<int(p.size());++i) {
                if((i&15)==0)checkpoint("initial");
                neighbours[i]=tree.query(p[i].p,i,8);
                for(int k=0;k<4;++k) {
                    Point d=p[neighbours[i].id[k]].p-p[i].p;
                    lengths.push_back(norm(d));
                    vectors.push_back(d);
                }
            }
            double pitch=med(lengths),cs=0,sn=0;
            if(pitch<min_pitch)return reject("INITIAL_GEOMETRY");
            std::vector<Point> keep;
            for(size_t i=0;i<vectors.size();++i)if(lengths[i]>.55*pitch&&lengths[i]<1.35*pitch) {
                Point d=vectors[i];
                const double xx=d.x*d.x,yy=d.y*d.y,z=xx+yy;
                const double invz2=1./(z*z);
                cs+=(xx*xx-6*xx*yy+yy*yy)*invz2;
                sn+=4*d.x*d.y*(xx-yy)*invz2;
                keep.push_back(d);
            }
            double theta=std::atan2(sn,cs)/4;
            b= {
                {
                    pitch*std::cos(theta),pitch*std::sin(theta)
                }, {
                    -pitch*std::sin(theta),pitch*std::cos(theta)
                },pitch
            };
            for(int it=0;it<3;++it) {
                std::vector<double> xx,xy,yx,yy;
                for(Point d:keep) {
                    Point q=b.inv(d);
                    int x=int(std::lround(q.x)),y=int(std::lround(q.y));
                    if(std::abs(x)+std::abs(y)!=1||norm(q-Point {
                        double(x),double(y)
                    })>=.48)continue;
                    if(x) {
                        xx.push_back(d.x*x);
                        xy.push_back(d.y*x);
                    }
                    else {
                        yx.push_back(d.x*y);
                        yy.push_back(d.y*y);
                    }
                }
                if(xx.size()<4||yx.size()<4)break;
                Basis next= {
                    {
                        med(xx),med(xy)
                    }, {
                        med(yx),med(yy)
                    },pitch
                };
                if(next.x.x*next.y.y-next.x.y*next.y.x < .1*pitch*pitch)break;
                b=next;
            }
            return true;
        }
        bool graph(const std::vector<Dot>& p,const Basis& b,const std::vector<Knn>& nn,std::vector<Point>& grid,std::vector<Point>& pix) {
            checkpoint("neighbors");
            const double determinant=b.x.x*b.y.y-b.x.y*b.y.x;
            if(!std::isfinite(determinant)||std::abs(determinant)<1e-12)return false;
            const double invdet=1./determinant;
            const double a00=b.y.y*invdet,a01=-b.y.x*invdet,a10=-b.x.y*invdet,a11=b.x.x*invdet;
            std::vector<Edge> edges;
            edges.reserve(p.size()*5);
            for(int i=0;i<int(p.size());++i)for(int k=0;k<nn[i].k;++k) {
                if(k==0 && (i&7)==0)checkpoint("neighbors");
                int j=nn[i].id[k];
                if(j<=i)continue;
                const Point d=p[j].p-p[i].p;
                const Point q{a00*d.x+a01*d.y,a10*d.x+a11*d.y};
                int dx=int(std::lround(q.x)),dy=int(std::lround(q.y));
                double e=norm2(q-Point {
                    double(dx),double(dy)
                });
                if(std::abs(dx)+std::abs(dy)==1&&e<.48*.48)edges.push_back( {
                    float(e),i,j,dx,dy
                });
            }
            std::sort(edges.begin(),edges.end(),[](const Edge&a,const Edge&b) {
                return a.error<b.error;
            });
            std::vector<Union> u(p.size());
            for(int i=0;i<int(p.size());++i)u[i]= {
                i,1,0,0
            };
            for(const auto&e:edges) {
                int a=ufroot(u,e.a),b0=ufroot(u,e.b);
                if(a==b0)continue;
                int dx=e.dx+u[e.a].x-u[e.b].x,dy=e.dy+u[e.a].y-u[e.b].y;
                if(u[a].size>=u[b0].size) {
                    u[b0].parent=a;
                    u[b0].x=dx;
                    u[b0].y=dy;
                    u[a].size+=u[b0].size;
                }
                else {
                    u[a].parent=b0;
                    u[a].x=-dx;
                    u[a].y=-dy;
                    u[b0].size+=u[a].size;
                }
            }
            int best=0;
            for(int i=0;i<int(p.size());++i) {
                if((i&15)==0)checkpoint("neighbors");
                int r=ufroot(u,i);
                if(u[r].size>u[best].size)best=r;
            }
            grid.clear();
            pix.clear();
            for(int i=0;i<int(p.size());++i)if(ufroot(u,i)==best) {
                grid.push_back( {
                    double(u[i].x),double(u[i].y)
                });
                pix.push_back(p[i].p);
            }
            return grid.size()>=64;
        }
        bool fit_grid(const std::vector<Point>& base,const std::vector<Point>& pix,double pitch,Mat& h) {
            checkpoint("fit_grid");
            if(!fitH(base,pix,h,nullptr,&budget))return reject("GRID_FIT");
            std::vector<double> weights(base.size(),1.);
            std::vector<Point> source,target;
            // Robust initial residuals before assigning encoded displacements.
            for(int it=0;it<2;++it) {
                checkpoint("fit_grid");
                std::vector<double> errors;
                for(size_t i=0;i<base.size();++i)errors.push_back(norm(h.apply(base[i])-pix[i]));
                double scale=std::max(.2*pitch,med(errors)*2.5);
                for(size_t i=0;i<base.size();++i)weights[i]=1/(1+sq(errors[i]/scale));
                if(!fitH(base,pix,h,&weights,&budget))return reject("GRID_FIT");
            }
            const double ratio=o.offset_mm/o.pitch_mm;
            std::vector<int> associations,last_associations,older_associations;
            Mat older_solution;
            associations.reserve(base.size()*2);last_associations.reserve(base.size()*2);
            for(int it=0;it<18;++it){
                checkpoint("fit_grid");
                Mat inv;if(!h.inverse(inv))return reject("GRID_FIT");
                source.clear();target.clear();associations.clear();
                const double limit=it<2?.30:it<5?.18:.12;
                for(size_t i=0;i<base.size();++i){
                if((i&15u)==0)checkpoint("fit_grid");
                    double error2;const int symbol=classify_squared(inv.apply(pix[i])-base[i],ratio,error2);
                    if(error2<limit*limit){
                        source.push_back(base[i]+offset(symbol,ratio));target.push_back(pix[i]);
                        associations.push_back(int(i));associations.push_back(symbol);
                    }
                }
                if(source.size()<24)return reject("GRID_FIT");
                // Still evaluate the tighter gates. Identical selected pairs have
                // the identical LS solution, so skip only the redundant solve.
                if(associations==last_associations){if(it>=5)break;continue;}
                if(it>=7 && associations==older_associations){
                    // Same ordered pairs -> exactly the same LS system. Fast-forward
                    // to iteration 18's state; do NOT accept an unverified estimate.
                    if((18-it)&1)h=older_solution;
                    return true;
                }
                Mat next;if(!fitH(source,target,next,nullptr,&budget))return reject("GRID_FIT");
                older_solution=h;older_associations=last_associations;
                h=next;last_associations=associations;
            }
            return true;
        }
        bool hint_fit(const std::vector<Dot>& p,Mat& h) {
            checkpoint("hint");
            const double ratio=o.offset_mm/o.pitch_mm;
            previous_associations.clear();
            for(double limit:{.22,.16,.12,.10}){
                checkpoint("hint");
                Mat inv;if(!h.inverse(inv))return reject("PRIOR_GEOMETRY");
                fit_src.clear();fit_dst.clear();fit_associations.clear();
                for(size_t i=0;i<p.size();++i){
                if((i&15u)==0)checkpoint("hint");
                    Point q=inv.apply(p[i].p);
                    if(!std::isfinite(q.x)||!std::isfinite(q.y)||std::abs(q.x)>1e5||std::abs(q.y)>1e5)return reject("PRIOR_GEOMETRY");
                    int gx=int(std::lround(q.x)),gy=int(std::lround(q.y));
                    Point g{double(gx),double(gy)};
                    double error2;int symbol=classify_squared(q-g,ratio,error2);
                    if(error2<limit*limit){
                        fit_src.push_back(g+offset(symbol,ratio));fit_dst.push_back(p[i].p);
                        fit_associations.insert(fit_associations.end(),{int(i),gx,gy,symbol});
                    }
                }
                if(fit_src.size()<64)return reject("PRIOR_GEOMETRY");
                if(fit_associations!=previous_associations){
                    if(!fitH(fit_src,fit_dst,h,nullptr,&budget))return reject("PRIOR_GEOMETRY");
                    previous_associations=fit_associations;
                }
            }
            return true;
        }
        bool build_grid(const std::vector<Dot>& p,const Mat& input,Grid& g,Observation* diagnostic=nullptr) {
            checkpoint("grid_check");
            Mat inv;
            if(!input.inverse(inv))return reject("SINGULAR_GRID");
            double ratio=o.offset_mm/o.pitch_mm;
            std::vector<int> cx(p.size()),cy(p.size()),sym(p.size());
            std::vector<double> err(p.size()),margin(p.size());
            int minx=1000000,miny=1000000,maxx=-1000000,maxy=-1000000,n=0;
            for(size_t i=0;i<p.size();++i) {
                if((i&15u)==0)checkpoint("grid_check");
                Point q=inv.apply(p[i].p);
                if(!std::isfinite(q.x)||!std::isfinite(q.y)||std::abs(q.x)>1e5||std::abs(q.y)>1e5)return reject("INVALID_GRID_POINT");
                cx[i]=int(std::lround(q.x));
                cy[i]=int(std::lround(q.y));
                sym[i]=classify(q-Point {
                    double(cx[i]),double(cy[i])
                },ratio,err[i],margin[i]);
                if(err[i]<.075&&margin[i]>.05) {
                    ++n;
                    minx=std::min(minx,cx[i]);
                    maxx=std::max(maxx,cx[i]);
                    miny=std::min(miny,cy[i]);
                    maxy=std::max(maxy,cy[i]);
                }
            }
            if(n<64)return reject("FEW_GRID_POINTS");
            if(n<double(p.size())*.65)return reject("GRID_SUPPORT");
            if(med(err)>.055)return reject("GRID_RESIDUAL");
            g.w=maxx-minx+1;
            g.height=maxy-miny+1;
            if(g.w<8||g.height<8)return reject("GRID_TOO_SMALL");
            if(g.w>256||g.height>256||g.w*g.height>8192)return reject("GRID_SPAN");
            g.cells.assign(size_t(g.w)*g.height,Cell {
            });
            g.points.clear();
            g.symbols.clear();
            std::vector<int> counts(g.cells.size(),0);
            for(size_t i=0;i<p.size();++i) {
                if((i&15u)==0)checkpoint("grid_check");
                int x=cx[i]-minx,y=cy[i]-miny;
                if(x>=0&&y>=0&&x<g.w&&y<g.height&&err[i]<.18)++counts[size_t(y)*g.w+x];
            }
            for(size_t i=0;i<p.size();++i) {
                if((i&15u)==0)checkpoint("grid_check");
                int x=cx[i]-minx,y=cy[i]-miny;
                if(x<0||y<0||x>=g.w||y>=g.height)continue;
                size_t idx=size_t(y)*g.w+x;
                if(counts[idx]!=1||err[i]>=.075||margin[i]<=.05)continue;
                g.cells[idx]= {
                    int(g.points.size()),std::uint8_t(sym[i]),float(err[i])
                };
                g.points.push_back(p[i].p);
                g.symbols.push_back(std::uint8_t(sym[i]));
            }
            Mat shift= {
                {
                    1,0,double(minx),0,1,double(miny),0,0,1
                }
            };
            g.h=mul(input,shift);
            g.median_error=med(err);
            if(diagnostic){
                double sum=0;int count=0;
                for(size_t i=0;i<p.size();++i)if(err[i]<.075&&margin[i]>.05){sum+=err[i]*err[i];++count;}
                diagnostic->rigid_residual_cells=count?std::sqrt(sum/count):0.;
            }
            return g.points.size()>=64 || reject("FEW_UNIQUE_POINTS");
        }
        bool decode(Grid& g,FitAnswer& answer,Observation& obs) {
            StageScope stage_timer(accounting,obs.time.address_ms);

            checkpoint("decode");++decode_calls;
            struct Candidate {
                int r,c;
                double score;
            };
            std::vector<Candidate> blocks;
            for(int r=0;r<=g.height-8;++r)for(int c=0;c<=g.w-8;++c) {
                if((c&7)==0)checkpoint("decode");
                double score=0;
                bool valid=true;
                for(int y=0;y<8&&valid;++y)for(int x=0;x<8;++x) {
                    const auto& v=g.cells[size_t(r+y)*g.w+c+x];
                    if(v.id<0) {
                        valid=false;
                        break;
                    }
                    score+=v.error;
                }
                if(valid)blocks.push_back( {
                    r,c,score
                });
            }
            auto cmp=[](const Candidate&a,const Candidate&b) {
                return a.score<b.score;
            };
            if(int(blocks.size())>o.max_blocks) {
                std::nth_element(blocks.begin(),blocks.begin()+o.max_blocks,blocks.end(),cmp);
                blocks.resize(o.max_blocks);
            }
            std::sort(blocks.begin(),blocks.end(),cmp);
            bool found=false;
            std::int64_t tx0=0,ty0=0;
            int rot0=-1;
            const std::uint32_t maxx=std::uint32_t(std::floor(o.floor_width_mm/o.pitch_mm))+1,maxy=std::uint32_t(std::floor(o.floor_height_mm/o.pitch_mm))+1;
            for(const auto& b:blocks) {
                checkpoint("decode");
                std::uint8_t bits[64];
                for(int r=0;r<8;++r)for(int c=0;c<8;++c)bits[r*8+c]=g.cells[size_t(b.r+r)*g.w+b.c+c].symbol;
                Address a;
                if(!Codec::decode8(bits,a,o.section_x,o.section_y,maxx,maxy))continue;
                int rx=1,ry=0,sx=0,sy=1,bx=b.c,by=b.r;
                switch(a.quarter_turn_ccw) {
                    case 1:rx=0;
                    ry=-1;
                    sx=1;
                    sy=0;
                    by+=7;
                    break;
                    case 2:rx=-1;
                    ry=0;
                    sx=0;
                    sy=-1;
                    bx+=7;
                    by+=7;
                    break;
                    case 3:rx=0;
                    ry=1;
                    sx=-1;
                    sy=0;
                    bx+=7;
                    break;
                }
                std::int64_t tx=bx-std::int64_t(rx)*a.x-std::int64_t(sx)*a.y,ty=by-std::int64_t(ry)*a.x-std::int64_t(sy)*a.y;
                if(!found) {
                    answer.addr=a;
                    answer.block_r=b.r;
                    answer.block_c=b.c;
                    answer.blocks=1;
                    answer.rx=rx;
                    answer.ry=ry;
                    answer.sx=sx;
                    answer.sy=sy;
                    answer.bx=bx;
                    answer.by=by;
                    tx0=tx;
                    ty0=ty;
                    rot0=a.quarter_turn_ccw;
                    found=true;
                }
                else if(tx!=tx0||ty!=ty0||int(a.quarter_turn_ccw)!=rot0||a.section_x!=answer.addr.section_x||a.section_y!=answer.addr.section_y) {
                    obs.reason="CONFLICTING_8X8_ADDRESSES";
                    last_rejection="CODE_CONFLICT";

                    return false;
                }
                else ++answer.blocks;
                if(answer.blocks>=o.target_confirm)break;
            }

            if(!found||answer.blocks<o.min_confirm) {
                obs.reason=blocks.empty()?"NO_COMPLETE_8X8":"NO_EXACT_CODE_OR_TOO_FEW_CONFIRMATIONS";
                last_rejection=blocks.empty()?"NO_8X8":"NO_EXACT_CODE_OR_CONFIRM";
                return false;
            }
            answer.grid=std::move(g);
            return true;
        }
        void choose(Point centre,int count) {
            checkpoint("choose");
            // Retain only the closest bounded fitting set. Never copy the whole
            // detector pool into this vector (important for small MCU budgets).
            selected.clear();
            count=std::min(count,int(dots.size()));
            auto nearer=[&](const Dot& a,const Dot& b) {
                return norm2(a.p-centre)<norm2(b.p-centre);
            };
            for(const auto& d:dots) {
                if(int(selected.size())<count) {
                    selected.push_back(d);
                    std::push_heap(selected.begin(),selected.end(),nearer);
                } else if(count>0 && nearer(d,selected.front())) {
                    std::pop_heap(selected.begin(),selected.end(),nearer);
                    selected.back()=d;
                    std::push_heap(selected.begin(),selected.end(),nearer);
                }
            }
        }
        bool cold(Point centre,int count,int variants,FitAnswer& ans,Observation& obs) {
            StageScope stage_timer(accounting,obs.time.geometry_ms);
            if(!recovery_allowed())return false;
            choose(centre,count);
            cold_key.clear();for(const auto& d:selected)cold_key.push_back(d.p);
            std::sort(cold_key.begin(),cold_key.end(),[](Point a,Point b){
                return a.x<b.x || (a.x==b.x && a.y<b.y);
            });
            checkpoint("cold_key");
            bool same=cold_key.size()==last_cold_key.size();
            for(size_t i=0;same && i<cold_key.size();++i)
                same=cold_key[i].x==last_cold_key[i].x && cold_key[i].y==last_cold_key[i].y;
            if(!same){last_cold_key=cold_key;cold_variants_done=0;}
            variants=std::clamp(variants,1,7);
            const unsigned wanted=(1u<<variants)-1u;
            if((cold_variants_done&wanted)==wanted){++duplicate_candidates;return false;}
            Basis base;std::vector<Knn> nn;
            if(!initial(selected,base,nn)){
                cold_variants_done=0x7fu;return reject("INITIAL_GEOMETRY");
            }
            const double angle[]={0,-4,4,0,0,-8,8},scale[]={1,1,1,.9,1.1,1,1};
            for(int k=0;k<variants;++k){
                const unsigned bit=1u<<k;
                if(cold_variants_done&bit){++duplicate_candidates;continue;}
                if(!recovery_allowed())break;
                cold_variants_done|=bit;++recovery_count;++obs.fit_attempts;
                const double a=angle[k]*pi/180,s=scale[k];
                const double cs=std::cos(a),sn=std::sin(a);
                auto rot=[&](Point p){return Point{s*(cs*p.x-sn*p.y),s*(sn*p.x+cs*p.y)};};
                Basis basis{rot(base.x),rot(base.y),base.pitch};
                std::vector<Point> q,p;Mat h;Grid g;
                if(!graph(selected,basis,nn,q,p)){reject("GRID_CONNECTIVITY");continue;}
                if(!fit_grid(q,p,basis.pitch,h)){reject("GRID_FIT");continue;}
                if(!build_grid(selected,h,g)){reject("GRID_VALIDATION");continue;}
                if(decode(g,ans,obs))return true;
            }
            return false;
        }
        // -------- Fixed-camera geometry: C is learned once; only SE(2) changes. --------
        void clear_tracking() {
            has_previous=false;
            has_track=false;
            history_count=0;
        }
        bool validate_profile(const CameraProfile& p) const {
            if(!p.valid||p.width<16||p.height<16||p.width>16000||p.height>16000||
               p.threshold<1||p.threshold>253||p.area_px<1||p.diameter_px<1||p.pitch_px<3.5||
               !std::isfinite(p.area_px)||!std::isfinite(p.diameter_px)||!std::isfinite(p.pitch_px)||
               std::abs(p.pitch_mm-o.pitch_mm)>1e-9||std::abs(p.offset_mm-o.offset_mm)>1e-9||
               !std::isfinite(p.reference.x)||!std::isfinite(p.reference.y)||
               p.reference.x<0||p.reference.x>=p.width||p.reference.y<0||p.reference.y>=p.height)
                return false;
            Mat c,ci;
            for(int i=0;i<9;++i){c.a[i]=p.image_from_metric[i];if(!std::isfinite(c.a[i]))return false;}
            if(!c.inverse(ci))return false;
            Point origin=c.apply({0,0});
            if(norm(origin-p.reference)>.05)return false;
            return true;
        }
        bool load_profile(const CameraProfile& p) {
            if(!validate_profile(p))return false;
            profile=p;
            for(int i=0;i<9;++i)cal.a[i]=p.image_from_metric[i];
            cal.inverse(cal_inv);
            clear_tracking();
            return true;
        }
        // The source and target are metric plane coordinates (one unit = print pitch).
        // Closed-form 2-D rigid registration: no 8x8 normal equations, no scale change.
        bool fit_rigid_pairs(const std::vector<Point>& src,const std::vector<Point>& dst,Mat& t) {
            checkpoint("rigid");
            if(src.size()!=dst.size()||src.size()<8)return false;
            Point a{},b{};
            for(size_t i=0;i<src.size();++i){
                if((i&15u)==0)checkpoint("rigid");
                a=a+src[i];b=b+dst[i];
            }
            a=a*(1./src.size());b=b*(1./src.size());
            double dot=0,cross=0,spread=0;
            for(size_t i=0;i<src.size();++i){
                if((i&15u)==0)checkpoint("rigid");
                Point u=src[i]-a,v=dst[i]-b;
                dot+=u.x*v.x+u.y*v.y;cross+=u.x*v.y-u.y*v.x;
                spread+=norm2(u);
            }
            double mag=std::hypot(dot,cross);
            if(mag<1e-10||spread<2)return false;
            double cs=dot/mag,sn=cross/mag;
            t=Mat{{cs,-sn,b.x-cs*a.x+sn*a.y,
                   sn,cs,b.y-sn*a.x-cs*a.y,0,0,1}};
            return true;
        }
        bool rigid_fit_reference(const std::vector<Dot>& p,Mat& h,Observation& obs) {
            checkpoint("rigid");
            if(!profile.valid||p.size()<64)return false;
            rectified.clear();
            for(const auto& d:p)rectified.push_back(cal_inv.apply(d.p));
            const double ratio=o.offset_mm/o.pitch_mm;
            // Associations are re-observed, not read from an expected bit string.
            for(double limit:{.23,.16,.11,.085}){
                checkpoint("rigid");
                ++obs.rigid_iterations;
                Mat inv;if(!h.inverse(inv))return false;
                fit_src.clear();fit_dst.clear();
                for(size_t i=0;i<p.size();++i){
                if((i&15u)==0)checkpoint("rigid");
                    Point q=inv.apply(p[i].p),g{std::round(q.x),std::round(q.y)};
                    double e,m;int s=classify(q-g,ratio,e,m);
                    if(e<limit){fit_src.push_back(g+offset(s,ratio));fit_dst.push_back(rectified[i]);}
                }
                Mat t;
                if(fit_src.size()<64||!fit_rigid_pairs(fit_src,fit_dst,t))return false;
                ++obs.rigid_solves;
                h=mul(cal,t);
            }
            // Diagnostic geometric residual is NOT a physical accuracy certificate.
            Mat inv;if(!h.inverse(inv))return false;
            double sum=0;int n=0;
            for(const auto& d:p){Point q=inv.apply(d.p),g{std::round(q.x),std::round(q.y)};
                double e,m;classify(q-g,ratio,e,m);if(e<.075&&m>.05){sum+=e*e;++n;}}
            if(n<64)return false;
            obs.rigid_residual_cells=std::sqrt(sum/n);
            return true;
        }
        bool rigid_fit(const std::vector<Dot>& p,Mat& h,Observation& obs){
            checkpoint("rigid");
            if(!o.fast_rigid)return rigid_fit_reference(p,h,obs);
            if(!profile.valid||p.size()<64)return reject("RIGID_FIT");
            rectified.clear();
            for(const auto& d:p)rectified.push_back(cal_inv.apply(d.p));
            Mat t=mul(cal_inv,h);
            const double ratio=o.offset_mm/o.pitch_mm;
            previous_associations.clear();
            bool is_rigid=false;
            for(double limit:{.23,.16,.11,.085}){
                checkpoint("rigid");
                ++obs.rigid_iterations;
                Mat inv;
                if(is_rigid){
                    // Inverse of the last SE(2) fit: no general 3x3 inverse and
                    // no projective division in the point loop.
                    inv=Mat{{t.a[0],t.a[3],-t.a[0]*t.a[2]-t.a[3]*t.a[5],
                             t.a[1],t.a[4],-t.a[1]*t.a[2]-t.a[4]*t.a[5],0,0,1}};
                }else if(!t.inverse(inv))return reject("RIGID_FIT");
                fit_src.clear();fit_dst.clear();fit_associations.clear();
                const double limit2=limit*limit;
                for(size_t i=0;i<p.size();++i){
                if((i&15u)==0)checkpoint("rigid");
                    const auto z=rectified[i];
                    const Point q=is_rigid?Point{inv.a[0]*z.x+inv.a[1]*z.y+inv.a[2],
                                               inv.a[3]*z.x+inv.a[4]*z.y+inv.a[5]}:inv.apply(z);
                    const int gx=int(std::lround(q.x)),gy=int(std::lround(q.y));
                    const Point g{double(gx),double(gy)};
                    double e2;const int symbol=classify_squared(q-g,ratio,e2);
                    if(e2<limit2){
                        fit_src.push_back(g+offset(symbol,ratio));fit_dst.push_back(z);
                        fit_associations.push_back(int(i));fit_associations.push_back(gx);
                        fit_associations.push_back(gy);fit_associations.push_back(symbol);
                    }
                }
                if(fit_src.size()<64)return reject("RIGID_FIT");
                // Identical ordered associations give EXACTLY the same least-
                // squares pairs; reuse the previous fit, not a looser gate.
                // Every stricter threshold is still evaluated.
                if(fit_associations!=previous_associations){
                    if(!fit_rigid_pairs(fit_src,fit_dst,t))return reject("RIGID_FIT");
                    ++obs.rigid_solves;
                    previous_associations=fit_associations;
                }
                is_rigid=true;
            }
            h=mul(cal,t);
            // build_grid immediately follows every caller and already applies the
            // exact .075/.05 error+margin gates, requires >=64 unambiguous points
            // and checks median/valid fraction. It also computes the diagnostic.
            // Do not classify all points for a redundant second final pass here.
            return true;
        }
        // Translation/rotation can change arbitrarily between frames. Once optics
        // are known, rebuild CURRENT adjacency in rectified metric coordinates,
        // then solve ONLY rigid motion, with no previous-address constraint.
        bool reacquire_fixed(Point ref,FitAnswer& answer,Observation& obs){
            StageScope stage_timer(accounting,obs.time.geometry_ms);
            if(!o.cached_reacquire||!profile.valid||!recovery_allowed())return false;
            choose(ref,o.max_fit_points);
            if(selected.size()<64)return false;

            ++recovery_count;++obs.fit_attempts;
            std::vector<Dot> rect=selected;
            for(auto& d:rect)d.p=cal_inv.apply(d.p);
            Basis base;std::vector<Knn> nn;std::vector<Point> q,z;
            Mat t,h;Grid g;
            bool ok=initial(rect,base,nn,.2)&&graph(rect,base,nn,q,z)&&fit_rigid_pairs(q,z,t);
            if(ok){h=mul(cal,t);ok=rigid_fit(selected,h,obs)&&build_grid(selected,h,g,&obs);}

            if(!ok||!decode(g,answer,obs))return false;
            obs.used_cached_appearance=true;obs.used_rigid_grid=true;
            obs.used_fixed_reacquire=true;obs.threshold="cached_reacquire";
            return true;
        }
        // v7: preserve the last verified optical shape, but let a small local
        // affine correction explain hand-held scale/tilt changes. A fit alone is
        // NEVER accepted; build_grid gates AND the current exact 8x8 decode follow.
        bool adaptive_fit(const std::vector<Dot>& p,Mat& h){
            checkpoint("affine");
            if(!o.adaptive_reuse||!profile.valid||p.size()<64)return reject("AFFINE_FIT");
            rectified.clear();for(const auto& d:p)rectified.push_back(cal_inv.apply(d.p));
            Mat t=mul(cal_inv,h);const double ratio=o.offset_mm/o.pitch_mm;
            previous_associations.clear();
            for(double limit:{.23,.16,.11,.085}){
                checkpoint("affine");
                Mat inv;if(!t.inverse(inv))return reject("AFFINE_FIT");
                fit_src.clear();fit_dst.clear();fit_associations.clear();
                for(size_t i=0;i<p.size();++i){
                if((i&15u)==0)checkpoint("affine");
                    Point q=inv.apply(rectified[i]);
                    if(!std::isfinite(q.x)||!std::isfinite(q.y)||std::abs(q.x)>1e5||std::abs(q.y)>1e5)return reject("AFFINE_FIT");
                    int gx=int(std::lround(q.x)),gy=int(std::lround(q.y));
                    Point g{double(gx),double(gy)};double error2;
                    int symbol=classify_squared(q-g,ratio,error2);
                    if(error2<limit*limit){
                        fit_src.push_back(g+offset(symbol,ratio));fit_dst.push_back(rectified[i]);
                        fit_associations.insert(fit_associations.end(),{int(i),gx,gy,symbol});
                    }
                }
                if(fit_src.size()<64)return reject("AFFINE_FIT");
                if(fit_associations!=previous_associations){
                    if(!fitAffine(fit_src,fit_dst,t,&budget))return reject("AFFINE_FIT");
                    previous_associations=fit_associations;
                }
            }
            h=mul(cal,t);return true;
        }
        bool adaptive_try(const std::vector<Dot>& p,const Mat& guess,FitAnswer& answer,Observation& obs){
            StageScope stage_timer(accounting,obs.time.geometry_ms);
            if(!o.adaptive_reuse)return false;
            Mat h=guess;Grid g;++obs.fit_attempts;
            bool fitted=adaptive_fit(p,h);
            bool ok=fitted&&build_grid(p,h,g,&obs);

            if(ok&&decode(g,answer,obs)){
                obs.used_adaptive_grid=true;return true;
            }
            // Only refine projectively if the affine correspondence stage
            // succeeded. Do not run an expensive solve on unsupported guesses.
            if(!fitted)return false;
            ++obs.fit_attempts;
            ok=hint_fit(p,h)&&build_grid(p,h,g,&obs);

            if(ok&&decode(g,answer,obs)){
                obs.used_adaptive_grid=true;return true;
            }
            return false;
        }
        // Recover a sub-cell translation after skipped frames WITHOUT tracking
        // the same physical dots. A tiny histogram supplies a seed only. U/D/L/R
        // are re-read and all final geometric/code checks remain unchanged.
        bool phase_try(const std::vector<Dot>& p,const Mat& hint,FitAnswer& answer,Observation& obs){
            StageScope stage_timer(accounting,obs.time.geometry_ms);
            if(!o.adaptive_reuse||p.size()<64)return false;
            Mat inv;
            if(!hint.inverse(inv)){return false;}
            constexpr int N=64;std::array<int,N> hx{},hy{};
            auto bin=[](double q){double f=q-std::floor(q);return int(std::floor(f*N+.5))&(N-1);};
            for(const auto& d:p){
                Point q=inv.apply(d.p);
                if(!std::isfinite(q.x)||!std::isfinite(q.y)||std::abs(q.x)>1e5||std::abs(q.y)>1e5){return false;}
                ++hx[bin(q.x)];++hy[bin(q.y)];
            }
            const int off=int(std::lround(N*o.offset_mm/o.pitch_mm));
            auto peaks=[&](const std::array<int,N>& hist){
                std::array<int,N> sm{};for(int j=0;j<N;++j)for(int k=-2;k<=2;++k)sm[j]+=hist[(j+k+N)&(N-1)];
                int first=0,best=-1;
                for(int j=0;j<N;++j){int score=2*sm[j]+sm[(j+off)&(N-1)]+sm[(j-off+N)&(N-1)];
                    if(score>best){best=score;first=j;}}
                return first<=N/2?double(first)/N:double(first-N)/N;
            };
            const double x=peaks(hx),y=peaks(hy);
            Mat shift{{1,0,x,0,1,y,0,0,1}},seed=mul(hint,shift);

            if(std::abs(x)+std::abs(y)<.03)return false;
            if(adaptive_try(p,seed,answer,obs)){
                obs.used_phase_alignment=true;obs.used_previous=true;
                obs.used_cached_appearance=true;obs.threshold="phase_aligned";return true;
            }
            return false;
        }
        // A translated periodic lattice may have almost no common physical dots.
        // Reuse its local axes (not old dot IDs) to build CURRENT integer adjacency.
        // This avoids re-estimating angles for every neighbour in the cold path.
        bool reacquire_basis(Point ref,const Mat& hint,FitAnswer& answer,Observation& obs){
            StageScope stage_timer(accounting,obs.time.geometry_ms);
            if(!o.adaptive_reuse||!profile.valid||selected.size()<64||!recovery_allowed())return false;
            ++recovery_count;++obs.fit_attempts;
            Mat inv;if(!hint.inverse(inv)){return false;}
            Point q0=inv.apply(ref);
            Point dx=derivative(hint,q0,{1,0}),dy=derivative(hint,q0,{0,1});
            Basis b{dx,dy,.5*(norm(dx)+norm(dy))};
            kd.set(selected);std::vector<Knn> nn;nn.reserve(selected.size());
            for(size_t i=0;i<selected.size();++i)nn.push_back(kd.query(selected[i].p,int(i),8));
            std::vector<Point> q,p,z;Mat t,h;
            bool ok=graph(selected,b,nn,q,p);
            if(ok){
                z.reserve(p.size());for(auto v:p)z.push_back(cal_inv.apply(v));
                ok=fitAffine(q,z,t,&budget);if(ok)h=mul(cal,t);
            }

            if(ok&&adaptive_try(selected,h,answer,obs)){
                obs.used_basis_reacquire=true;obs.used_cached_appearance=true;
                obs.used_previous=has_track;obs.threshold="cached_basis";return true;
            }
            return false;
        }
        Point camera_offset(double heading_deg) const {
            double a=heading_deg*pi/180;
            return {std::sin(a)*o.camera_forward_mm+std::cos(a)*o.camera_right_mm,
                    -std::cos(a)*o.camera_forward_mm+std::sin(a)*o.camera_right_mm};
        }
        Mat prediction(const MotionHint* hint,Observation& obs) {
            if(!has_track)return tracked;
            double turn=0,ds=0;Point next=last_camera;bool arc=false;
            if(hint&&hint->valid){
                if(!std::isfinite(hint->forward_mm)||!std::isfinite(hint->turn_deg))
                    throw std::invalid_argument("NONFINITE_MOTION_HINT");
                ds=hint->forward_mm;turn=hint->turn_deg;arc=true;obs.used_motion_hint=true;
            }else if(history_count>=2&&o.use_motion_prediction){
                const double oldturn=std::remainder(last_heading-older_heading,360.);
                turn=oldturn*prediction_scale;
                Point r1=last_camera-camera_offset(last_heading),r0=older_camera-camera_offset(older_heading);
                Point d=r1-r0;
                const double mid=(older_heading+oldturn*.5)*pi/180;
                Point f{std::sin(mid),-std::cos(mid)},right{std::cos(mid),std::sin(mid)};
                double along=d.x*f.x+d.y*f.y,lateral=d.x*right.x+d.y*right.y;
                const double a=oldturn*pi/180,si=std::abs(a)<1e-8?1.:std::sin(a*.5)/(a*.5);
                if(o.nonholonomic_prediction&&std::abs(lateral)<=.4+.2*std::abs(along)&&std::abs(si)>.2){
                    ds=along/si*prediction_scale;arc=true;
                }else{
                    obs.nonholonomic_relaxed=o.nonholonomic_prediction;
                    next=last_camera+(last_camera-older_camera)*prediction_scale;
                }
                obs.used_motion_prediction=true;
            }else return tracked;
            if(arc){
                const double a=turn*pi/180,mid=(last_heading+turn*.5)*pi/180;
                const double sinc=std::abs(a)<1e-8?1.:std::sin(a*.5)/(a*.5);
                Point robot=last_camera-camera_offset(last_heading);
                next=robot+Point{std::sin(mid)*ds*sinc,-std::cos(mid)*ds*sinc}+camera_offset(last_heading+turn);
            }
            if(norm(next-last_camera)>o.max_predict_mm||std::abs(turn)>o.max_predict_deg){
                obs.used_motion_prediction=false;obs.used_motion_hint=false;return tracked;
            }
            Point q0=last_camera*(1/o.pitch_mm)-anchor,q1=next*(1/o.pitch_mm)-anchor;
            const double a=turn*pi/180,cs=std::cos(-a),sn=std::sin(-a);
            Mat delta{{cs,-sn,q0.x-cs*q1.x+sn*q1.y,sn,cs,q0.y-sn*q1.x-cs*q1.y,0,0,1}};
            return mul(tracked,delta);
        }
        // Bounded window sampling around virtual grid intersections. The patch
        // covers ALL four possible displacement directions: no predicted U/D/L/R
        // is used. Every returned centre comes from CURRENT image intensities.
        bool sample_grid(GrayView im,Point ref,const Mat& h,Observation& obs) {
            checkpoint("sample");
            StageScope stage_timer(accounting,obs.time.dots_ms);
            Mat inv;if(!h.inverse(inv))return false;
            Point qc=inv.apply(ref);
            int side=std::min(o.patch_side,int(std::floor(std::sqrt(double(o.max_fit_points)))));
            int x0=int(std::lround(qc.x))-side/2,y0=int(std::lround(qc.y))-side/2;
            dots.clear();
            double refdet=std::abs(cal.a[0]*cal.a[4]-cal.a[1]*cal.a[3]);
            Point cx=derivative(cal,{0,0},{1,0}),cy=derivative(cal,{0,0},{0,1});
            refdet=std::abs(cx.x*cy.y-cx.y*cy.x);
            if(refdet<1e-9)return false;
            for(int r=0;r<side;++r)for(int c=0;c<side;++c){
                checkpoint("sample");
                Point q{double(x0+c),double(y0+r)};
                const double den=h.a[6]*q.x+h.a[7]*q.y+h.a[8];
                if(std::abs(den)<1e-10)continue;
                const double invden=1./den;
                Point p{(h.a[0]*q.x+h.a[1]*q.y+h.a[2])*invden,
                        (h.a[3]*q.x+h.a[4]*q.y+h.a[5])*invden};
                Point vx{(h.a[0]-p.x*h.a[6])*invden,(h.a[3]-p.y*h.a[6])*invden};
                Point vy{(h.a[1]-p.x*h.a[7])*invden,(h.a[4]-p.y*h.a[7])*invden};
                double pitch=std::min(norm(vx),norm(vy));
                int radius=int(std::ceil(.40*pitch));
                if(radius<3||radius>64)continue;
                int px=int(std::lround(p.x)),py=int(std::lround(p.y));
                if(px-radius<1||py-radius<1||px+radius>=im.width-1||py+radius>=im.height-1)continue;
                ++obs.sampled_windows;
                int area=0,l=radius+1,t=radius+1,rr=-radius-1,bb=-radius-1;
                std::int32_t sx=0,sy=0;
                for(int y=-radius;y<=radius;++y){auto row=im.data+size_t(py+y)*im.stride;
                    for(int x=-radius;x<=radius;++x)if(row[px+x]<=profile.threshold){
                        ++area;sx+=x;sy+=y;l=std::min(l,x);rr=std::max(rr,x);t=std::min(t,y);bb=std::max(bb,y);
                    }}
                double factor=std::abs(vx.x*vy.y-vx.y*vy.x)/refdet;
                double expected=profile.area_px*factor;
                if(area<std::max(2.,expected*.30)||area>expected*2.8||
                   l<=-radius||rr>=radius||t<=-radius||bb>=radius||l>rr)continue;
                int bw=rr-l+1,bh=bb-t+1;
                if(double(area)/(bw*bh)<.24||bw>4*bh||bh>4*bw)continue;
                // Same intensity centroid as the generic detector, small local support.
                int mx=px+int(std::lround(double(sx)/area)),my=py+int(std::lround(double(sy)/area));
                int rad=std::max(2,std::min(int(.33*pitch),int(std::ceil(std::max(bw,bh)*.5))+2));
                if(mx-rad<0||my-rad<0||mx+rad>=im.width||my+rad>=im.height)continue;
                Point centre;
                if(o.integer_centroids){
                    if(!integer_centroid(im,mx,my,rad,centre))continue;
                }else{
                double left=0,right=0,top=0,bottom=0;
                for(int u=-rad;u<=rad;++u){
                    left+=im.data[size_t(my+u)*im.stride+mx-rad];right+=im.data[size_t(my+u)*im.stride+mx+rad];
                    top+=im.data[size_t(my-rad)*im.stride+mx+u];bottom+=im.data[size_t(my+rad)*im.stride+mx+u];}
                double den=2*rad+1;left/=den;right/=den;top/=den;bottom/=den;
                double bg=(left+right+top+bottom)*.25,mass=0,xx=0,yy=0;
                for(int y=-rad;y<=rad;++y)for(int x=-rad;x<=rad;++x){
                    double b=bg+(right-left)*x/(2*rad)+(bottom-top)*y/(2*rad);
                    double w=std::max(0.,b-im.data[size_t(my+y)*im.stride+mx+x]-2.);
                    mass+=w;xx+=x*w;yy+=y*w;
                }
                if(mass<20)continue;
                centre={mx+xx/mass,my+yy/mass};
                }
                if(norm2(centre-Point{px+double(sx)/area,py+double(sy)/area})>sq(std::max(1.2,std::max(bw,bh)*.25)))continue;
                Dot dot;dot.p=centre;dot.area=area;dot.pitch=pitch;
                dot.x0=px+l;dot.x1=px+rr;dot.y0=py+t;dot.y1=py+bb;
                dots.push_back(dot);
            }
            obs.dots=int(dots.size());
            return dots.size()>=64;
        }
        bool fixed_attempt(GrayView im,Point ref,const MotionHint* motion,FitAnswer& answer,Observation& obs) {
            StageScope stage_timer(accounting,obs.time.geometry_ms);
            if(!o.fixed_camera||!o.use_previous||!profile.valid)return false;
            Mat predicted=has_track?prediction(motion,obs):cal;
            if(has_track&&o.patch_tracking){
                bool enough=sample_grid(im,ref,predicted,obs);

                if(enough){
                    selected.assign(dots.begin(),dots.end());
                    Mat h=predicted;Grid g;++obs.fit_attempts;
                    bool ok=rigid_fit(selected,h,obs)&&build_grid(selected,h,g,&obs);

                    if(ok&&decode(g,answer,obs)){
                        obs.used_previous=true;obs.used_cached_appearance=true;
                        obs.used_rigid_grid=true;obs.used_patch_sampling=true;
                        obs.threshold="cached_patch";return true;
                    }
                    if(adaptive_try(selected,predicted,answer,obs)){
                        obs.used_previous=true;obs.used_cached_appearance=true;
                        obs.used_patch_sampling=true;obs.threshold="adaptive_patch";return true;
                    }
                }
            }
            detect(im,preferred,ref,obs,true);
            obs.dots=int(dots.size());
            cached_scan_ready=dots.size()>=64;
            if(!cached_scan_ready)return false;
            choose(ref,o.max_fit_points);
            for(int guess=0;has_track&&guess<2;++guess){
                // No point in trying the identical seed twice (common when no
                // velocity estimate is available or the frame is stationary).
                double diff=0;for(int j=0;j<9;++j)diff=std::max(diff,std::abs(predicted.a[j]-tracked.a[j]));
                if(guess==1&&diff<1e-10)break;
                const Mat seed=guess==0?predicted:tracked;
                Mat h=seed;Grid g;++obs.fit_attempts;
                bool ok=rigid_fit(selected,h,obs)&&build_grid(selected,h,g,&obs);

                if(ok&&decode(g,answer,obs)){
                    obs.used_previous=true;obs.used_cached_appearance=true;obs.used_rigid_grid=true;
                    obs.threshold="cached_full";return true;
                }
                if(adaptive_try(selected,seed,answer,obs)){
                    obs.used_previous=true;obs.used_cached_appearance=true;obs.threshold="adaptive_grid";return true;
                }
            }
            if(has_track&&phase_try(selected,tracked,answer,obs))return true;
            if(reacquire_basis(ref,has_track?tracked:cal,answer,obs))return true;
            return reacquire_fixed(ref,answer,obs);
        }
        // C is centred at the chosen image reference. It is a metric plane
        // rectifier learned from the print, NOT lens/3-D camera calibration.
        void learn_profile(const FitAnswer& a,GrayView im,Point ref,const Observation& obs) {
            Mat inv;if(!a.grid.h.inverse(inv))return;
            Point q=inv.apply(ref);
            Mat centre{{1,0,q.x,0,1,q.y,0,0,1}};
            cal=mul(a.grid.h,centre);cal.inverse(cal_inv);
            CameraProfile p;p.valid=true;p.width=im.width;p.height=im.height;p.reference=ref;
            p.pitch_mm=o.pitch_mm;p.offset_mm=o.offset_mm;p.threshold=std::clamp(last_threshold,1,253);
            std::copy(cal.a,cal.a+9,p.image_from_metric.begin());
            p.diameter_px=obs.dot_diameter_px;p.pitch_px=obs.pitch_px;
            // Median of observed component areas. No metric diameter is invented.
            std::vector<double> areas;
            for(const auto& d:dots)areas.push_back(double(d.area));
            p.area_px=med(areas);
            if(validate_profile(p))profile=p;
        }
        bool check_optics(const Mat& current,Point ref) {
            Mat inv;if(!current.inverse(inv))return false;
            Point q=inv.apply(ref);
            fit_src.clear();fit_dst.clear();
            for(double y:{-4.,0.,4.})for(double x:{-4.,0.,4.}){
                Point p=q+Point{x,y};fit_src.push_back(p);fit_dst.push_back(cal_inv.apply(current.apply(p)));
            }
            Mat t;if(!fit_rigid_pairs(fit_src,fit_dst,t))return false;
            double maxe=0;for(size_t i=0;i<fit_src.size();++i)maxe=std::max(maxe,norm(t.apply(fit_src[i])-fit_dst[i]));
            return maxe<.08;
        }
        void remember(const FitAnswer& a,GrayView im,Point ref,Observation& obs) {
            if(!o.fixed_camera)return;
            bool was_profile=profile.valid;
            if(!was_profile)learn_profile(a,im,ref,obs);
            obs.calibration_ready=profile.valid;
            if(!profile.valid)return;
            // A rigid fit is constrained to cal by construction. Its pixel fit
            // was checked against current dots; repeating a 9-point synthetic
            // rigid consistency fit adds no independent optics evidence.
            obs.optics_consistent=obs.used_rigid_grid||check_optics(a.grid.h,ref);
            // The address and every symbol of the current 8x8 have already been
            // checked by decode(). Updating this search prior is NOT accepting a
            // prediction or relaxing any symbol/coordinate validation threshold.
            bool rebased=false;
            if(was_profile&&!obs.optics_consistent&&o.adaptive_reuse&&a.blocks>=std::max(2,o.min_confirm)){
                learn_profile(a,im,ref,obs);rebased=profile.valid;obs.profile_rebased=rebased;
                history_count=0; // preserve the measured grid, not an old velocity
            }
            Mat global_to_local{{double(a.rx),double(a.sx),double(a.bx)-a.rx*double(a.addr.x)-a.sx*double(a.addr.y),
                                 double(a.ry),double(a.sy),double(a.by)-a.ry*double(a.addr.x)-a.sy*double(a.addr.y),0,0,1}};
            Mat world=mul(a.grid.h,global_to_local);
            if(!has_track){anchor={std::round(obs.x_mm/o.pitch_mm),std::round(obs.y_mm/o.pitch_mm)};history_count=0;}
            Mat shift{{1,0,anchor.x,0,1,anchor.y,0,0,1}};
            Mat h=mul(world,shift);
            if(has_track){older_camera=last_camera;older_heading=last_heading;older_tracked=tracked;}
            tracked=h;last_camera={obs.x_mm,obs.y_mm};last_heading=obs.heading_deg;
            has_track=true;history_count=std::min(2,history_count+1);
            // Exposure/size are reused while locked, refreshed only by a newly
            // fully verified general detection. Optical rectifier stays frozen.
            if(was_profile&&!obs.used_cached_appearance&&obs.optics_consistent){
                profile.threshold=std::clamp(last_threshold,1,253);
                profile.diameter_px=obs.dot_diameter_px;
                std::vector<double> areas;for(const auto& d:dots)areas.push_back(double(d.area));
                if(!areas.empty())profile.area_px=med(areas);
            }
            if(!was_profile)obs.status="CALIBRATED";
            else if(rebased)obs.status=obs.used_phase_alignment?"ADAPTIVE_PHASE":obs.used_basis_reacquire?"ADAPTIVE_REACQUIRED":
                obs.used_patch_sampling?"ADAPTIVE_PATCH":obs.used_adaptive_grid?"ADAPTIVE_GRID":"PROFILE_UPDATED";
            else if(!obs.optics_consistent){
                obs.status="FULL_OPTICS_CHANGED";
                if(!o.adaptive_reuse)clear_tracking(); // v6 behaviour only in fixed mode
                else history_count=1; // retain a code-verified previous grid
            }
            else if(obs.used_phase_alignment)obs.status="ADAPTIVE_PHASE";
            else if(obs.used_basis_reacquire)obs.status="ADAPTIVE_REACQUIRED";
            else if(obs.used_adaptive_grid)obs.status=obs.used_patch_sampling?"ADAPTIVE_PATCH":"ADAPTIVE_GRID";
            else if(obs.used_fixed_reacquire)obs.status="LOCKED_REACQUIRED";
            else if(obs.used_patch_sampling)obs.status="LOCKED_PATCH";
            else if(obs.used_rigid_grid)obs.status="LOCKED_GRID";
            else obs.status="REACQUIRED";
        }

        bool finish(FitAnswer& a,GrayView im,Point ref,Observation& obs) {
            checkpoint("finish");
            // Verification is complete. Publish the result and update the prior
            // together, without leaving a half-updated tracking state on timeout.
            // This bounded bookkeeping can extend the cooperative time limit.
            FinishBudgetPause commit_guard(budget);
            Mat inv;
            if(!a.grid.h.inverse(inv))return false;
            Point q=inv.apply(ref),delta=q-Point {
                double(a.bx),double(a.by)
            };
            double gx=a.addr.x+a.rx*delta.x+a.ry*delta.y,gy=a.addr.y+a.sx*delta.x+a.sy*delta.y;
            double angle=o.forward_image_deg*pi/180;
            Point v=derivative(inv,ref, {
                std::sin(angle),-std::cos(angle)
            });
            Point forward= {
                a.rx*v.x+a.ry*v.y,a.sx*v.x+a.sy*v.y
            };
            if(!std::isfinite(gx)||!std::isfinite(gy)||norm(forward)<1e-10)return false;
            obs.x_mm=gx*o.pitch_mm;
            obs.y_mm=gy*o.pitch_mm;
            obs.heading_deg=std::fmod(std::atan2(forward.x,-forward.y)*180/pi+360,360);
            obs.valid=true;
            obs.status=obs.used_previous?"FAST_VERIFIED":"VERIFIED";
            obs.reason.clear();
            obs.address=a.addr;
            obs.verified_blocks=a.blocks;
            obs.valid_dots=int(a.grid.points.size());
            Point corners[4]= {
                {
                    a.block_c-.45,a.block_r-.45
                }, {
                    a.block_c+7.45,a.block_r-.45
                }, {
                    a.block_c+7.45,a.block_r+7.45
                }, {
                    a.block_c-.45,a.block_r+7.45
                }
            };
            for(int i=0;i<4;++i)obs.box[i]=a.grid.h.apply(corners[i]);
            int minx=100000,maxx=-100000,miny=100000,maxy=-100000;
            for(int r=0;r<a.grid.height;++r)for(int c=0;c<a.grid.w;++c)if(a.grid.cells[size_t(r)*a.grid.w+c].id>=0) {
                minx=std::min(minx,c);
                maxx=std::max(maxx,c);
                miny=std::min(miny,r);
                maxy=std::max(maxy,r);
            }
            // Conservative bounding support, not a calibrated covariance / convex hull.
            obs.reference_supported=q.x>=minx&&q.x<=maxx&&q.y>=miny&&q.y<=maxy;
            std::vector<int> areas;areas.reserve(dots.size());
            for(const auto& d:dots)areas.push_back(d.area);
            if(!areas.empty()){
                const auto k=areas.size()/2;
                std::nth_element(areas.begin(),areas.begin()+k,areas.end());
                obs.dot_diameter_px=2*std::sqrt(double(areas[k])/pi);
                if(areas.size()%2==0)obs.dot_diameter_px=(obs.dot_diameter_px+
                    2*std::sqrt(double(*std::max_element(areas.begin(),areas.begin()+k))/pi))*.5;
            }
            Point p0=a.grid.h.apply(q),px=a.grid.h.apply(q+Point {
                1,0
            }),py=a.grid.h.apply(q+Point {
                0,1
            });
            obs.pitch_px=.5*(norm(px-p0)+norm(py-p0));
            for(int r=0;r<8;++r)for(int c=0;c<8;++c) {
                const Cell& cell=a.grid.cells[size_t(a.block_r+r)*a.grid.w+a.block_c+c];
                obs.selected_dots[r*8+c]=a.grid.points[cell.id];
                obs.selected_symbols[r*8+c]=cell.symbol;
            }
            previous=a.grid.h;
            has_previous=true;
            workspace_peak=std::max(workspace_peak,bytes()+selected.size()*640u+a.grid.cells.capacity()*sizeof(Cell)+a.grid.points.capacity()*sizeof(Point)+a.grid.symbols.capacity());
            obs.workspace_capacity_bytes=workspace_peak;
            remember(a,im,ref,obs);
            return true;
        }
        Observation process(GrayView im,Point ref,const MotionHint* motion=nullptr) {
            Observation obs;
            auto total=Clock::now();processing_start=total;recovery_count=0;cached_scan_ready=false;
            budget.start(total,o.recovery_budget_ms);
            last_rejection="NONE";decode_calls=0;duplicate_candidates=0;
            last_cold_key.clear();cold_variants_done=0;
            bool timed_out=false,runtime_error=false;const char* interrupted_stage=nullptr;
            if(!im.data||im.width<16||im.height<16||im.stride<im.width||im.width>16000||im.height>16000) {
                obs.reason="INVALID_IMAGE";
                return obs;
            }
            if(ref.x<0||ref.y<0)ref= {
                (im.width-1)*.5,(im.height-1)*.5
            };
            if(profile.valid && (im.width!=profile.width||im.height!=profile.height||norm(ref-profile.reference)>.01)) {
                clear_tracking();obs.reason="PROFILE_IMAGE_SIZE_OR_REFERENCE_MISMATCH";
                obs.time.total_ms=ms(total);return obs;
            }
            bool success=false;
            FitAnswer answer;
            try {
                if(fixed_attempt(im,ref,motion,answer,obs)&&finish(answer,im,ref,obs)) {
                    obs.time.total_ms=ms(total);return obs;
                }
                // Do not leave a partial fast-path flag on an independent recovery.
                obs.used_cached_appearance=false;obs.used_rigid_grid=false;obs.used_patch_sampling=false;obs.used_fixed_reacquire=false;obs.used_adaptive_grid=false;obs.used_basis_reacquire=false;obs.used_phase_alignment=false;
                int modes[3]= {
                    preferred,0,1
                };
                if(preferred==0) {
                    modes[1]=1;
                    modes[2]=2;
                }
                else if(preferred==1) {
                    modes[1]=0;
                    modes[2]=2;
                }
                else {
                    modes[1]=0;
                    modes[2]=1;
                }
                for(int stage=0;stage<3&&!success;++stage) {
                    if(!recovery_allowed())break;
                    int mode=modes[stage];

                    // The cached scan has already measured CURRENT intensities.
                    // Reuse it for this first general fit, not re-detect the same
                    // frame. Later thresholds still perform independent detection.
                    if(!(o.adaptive_reuse&&cached_scan_ready&&stage==0&&mode==preferred)){
                        detect(im,mode,ref,obs);
                    }
                    obs.dots=int(dots.size());
                    obs.threshold=mode==0?"otsu":mode==1?"local":"local_wide";
                    if(dots.size()<64) {
                        last_rejection="DOTS_LT64";
                        obs.reason="TOO_FEW_DOTS_FOR_8X8";
                        continue;
                    }
                    if(o.use_previous&&has_previous) {
                        choose(ref,o.max_fit_points);

                        ++obs.fit_attempts;
                        Mat h=previous;
                        Grid g;
                        bool ok;
                        {StageScope timer(accounting,obs.time.geometry_ms);
                         ok=hint_fit(selected,h)&&build_grid(selected,h,g);}
                        if(!ok)reject("PRIOR_GEOMETRY");
                        if(ok&&decode(g,answer,obs)) {
                            obs.used_previous=true;
                            success=true;
                        }
                    }
                    if(!success)success=cold(ref,o.max_fit_points,
                        o.max_fit_points<=240?(o.robust?7:3):1,answer,obs);
                    if(!success) {
                        std::vector<double> xx,yy;
                        for(const auto& d:dots) {
                            xx.push_back(d.p.x);
                            yy.push_back(d.p.y);
                        }
                        Point middle= {
                            med(xx),med(yy)
                        };
                        const Point centres[]= {
                            ref,middle, {
                                im.width*.5,im.height*.25
                            }, {
                                im.width*.5,im.height*.75
                            }, {
                                im.width*.25,im.height*.5
                            }, {
                                im.width*.75,im.height*.5
                            }
                        };
                        int count=std::min(o.max_fit_points,240),limit=o.robust?6:3;
                        for(int k=o.max_fit_points<=240?1:0;k<limit&&!success;++k)
                            success=cold(centres[k],count,o.robust?7:3,answer,obs);
                    }
                    if(success) {
                        preferred=mode;
                        success=finish(answer,im,ref,obs);
                    }
                }
            }
            catch(const FrameBudgetExceeded& e) {
                timed_out=true;interrupted_stage=e.stage;success=false;
            }
            catch(const std::exception& e) {
                runtime_error=true;
                obs.reason=e.what();
                success=false;
            }
            if(!success) {
                clear_tracking();
                obs.calibration_ready=profile.valid;
                obs.valid=false;
                obs.status="LOST";
                obs.reference_supported=false;obs.verified_blocks=0;
                obs.x_mm=obs.y_mm=obs.heading_deg=0.;
                if(timed_out)obs.reason=failure_reason("TIME_LIMIT",interrupted_stage);
                else if(!runtime_error && o.fixed_camera && recovery_count>=o.max_recovery_fits)
                    obs.reason=failure_reason("FIT_LIMIT");
                else if(obs.reason.empty())obs.reason=failure_reason("NO_VERIFIED_GRID");
            }
            obs.time.total_ms=ms(total);
            if(!obs.workspace_capacity_bytes)obs.workspace_capacity_bytes=bytes()+selected.size()*640u;
            return obs;
        }
    };
    Engine::Engine(const Options& o):impl_(new Impl(o)) {
    }
    Engine::~Engine()=default;
    Observation Engine::process(GrayView im,Point ref) {
        return impl_->process(im,ref);
    }
    void Engine::reset() {
        impl_->clear_tracking();
        impl_->profile=CameraProfile{};
        impl_->preferred=0;
    }
    void Engine::reset_tracking(){impl_->clear_tracking();}
    void Engine::clear_velocity_history(){impl_->history_count=std::min(impl_->history_count,1);}
    void Engine::set_prediction_scale(double ratio){
        impl_->prediction_scale=std::isfinite(ratio)&&ratio>=.25&&ratio<=3.?ratio:1.;
    }
    CameraProfile Engine::camera_profile() const {return impl_->profile;}
    bool Engine::set_camera_profile(const CameraProfile& p){return impl_->load_profile(p);}
    Observation Engine::process_with_motion(GrayView im,const MotionHint& hint,Point ref){
        return impl_->process(im,ref,&hint);
    }
}
// namespace anoto
