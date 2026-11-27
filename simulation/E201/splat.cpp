// Perspective-correct ray/surfel intersection with per-pixel front-to-back alpha.
#include <algorithm>
#include <cmath>
#include <vector>

struct Hit { float depth, alpha; int source; };
static float dot(const float *a,const float *b) {
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}
extern "C" void render_splats(const float *s, int count, const float *intr, int w, int h,
                              float *depth, float *rgb, float *opacity, float *spread, int *id) {
    const float fx=intr[0], fy=intr[1], cx=intr[2], cy=intr[3];
    std::vector<std::vector<Hit>> bins(w*h);
    for (int i=0;i<count;++i) {
        const float *p=s+18*i;
        const float *n=p+3, *u=p+6, *v=p+9;
        float a=p[12], b=p[13], alpha=p[14];
        if (p[2]<=0 || dot(n,p)>-.09f*p[2]) continue;
        float mxx=-1e10f,mxy=-1e10f,mnx=1e10f,mny=1e10f;
        for(int k : {-1,1}) for(int j : {-1,1}) {
            float q[3];
            for(int d=0;d<3;++d) q[d]=p[d]+2.75f*(k*a*u[d]+j*b*v[d]);
            if(q[2]<=0) continue;
            float x=fx*q[0]/q[2]+cx, y=fy*q[1]/q[2]+cy;
            mxx=std::max(mxx,x);mnx=std::min(mnx,x);
            mxy=std::max(mxy,y);mny=std::min(mny,y);
        }
        if(mnx>w-1 || mxx<0 || mny>h-1 || mxy<0) continue;
        const int x0=std::max(0,(int)std::floor(mnx)), x1=std::min(w-1,(int)std::ceil(mxx));
        const int y0=std::max(0,(int)std::floor(mny)), y1=std::min(h-1,(int)std::ceil(mxy));
        const float num=dot(p,n);
        for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x) {
            float d[3]={(x-cx)/fx,(y-cy)/fy,1.f};
            const float denom=dot(n,d);
            if (denom>=-.065f) continue;
            const float z=num/denom;
            if(z<=0) continue;
            float off[3];
            for(int k=0;k<3;++k) off[k]=z*d[k]-p[k];
            const float tu=dot(off,u)/a,tv=dot(off,v)/b, r2=tu*tu+tv*tv;
            if(r2>7.5625f) continue;
            float coverage=alpha*std::exp(-.5f*r2);
            if(coverage>.009f) bins[y*w+x].push_back({z,coverage,i});
        }
    }
    for(int i=0;i<w*h;++i) {
        auto &arr=bins[i];
        if(arr.empty()) continue;
        std::sort(arr.begin(),arr.end(),[](const Hit &a,const Hit &b){return a.depth<b.depth;});
        float T=1.f, mass=0.f, zd=0.f, dist=0.f, median=0.f, best=0.f;
        for(auto &q:arr) {
            float weight=T*q.alpha;
            T*=1-q.alpha;
            float const *s_i=s+18*q.source;
            rgb[3*i]+=weight*s_i[15]; rgb[3*i+1]+=weight*s_i[16];
            rgb[3*i+2]+=weight*s_i[17];
            dist+=2.f*weight*(mass*q.depth-zd);
            mass+=weight;zd+=weight*q.depth;
            if(median==0 && mass>=.5f) median=q.depth;
            if(weight>best) {best=weight;id[i]=q.source;}
            if(T<.005f) break;
        }
        opacity[i]=mass;
        spread[i]=dist/std::max(.0001f,mass*mass);
        for(int k=0;k<3;++k) rgb[3*i+k]/=mass;
        depth[i]=median>0?median:zd/mass;
    }
}
