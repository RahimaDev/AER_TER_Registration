#include <list>
#include <cmath>
#include <iostream>
#include<vector>
#include <numeric>
#include <fstream>
#include <cstring>

#include <pcl/io/ply_io.h>

#include <pcl/point_types.h>

#include <pcl/common/geometry.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
# define M_PI           3.14159265358979323846  /* pi */

struct Line
{Eigen::Vector3d  A;
    Eigen::Vector3d  B;
    Eigen::Vector3d  d;
    double length;
    int Id;
    bool processed=false;
};
typedef   std::pair<Line,Line> Match;
/////////////////////////////////////////////////////////////////////////
Eigen::Vector3d direct_vec(Eigen::Vector3d A, Eigen::Vector3d B)
{
    Eigen::Vector3d v=B-A;


    return v/v.norm();
}
///////////////////////////////////////////////////////////////////
double PointLineDistance(Eigen::Vector3d p, Line l)
{


    Eigen::Vector3d C=l.A-p;
    //std::cout<<"C="<<C<<std::endl;
    Eigen::Vector3d K=C.cross(l.d);
    //std::cout<<"K="<<K<<std::endl;
    //std::cout<<(K.norm()/l.d.norm())<<std::endl;
    double dist=(K.norm()/l.d.norm());
    if (std::isnan(dist))
        return 0;
    else
        return dist;
}
double Line_To_Line_dist(Line l1,Line l2)
{
    Eigen::Vector3d g1=Eigen::Vector3d((l1.A.x()+l1.B.x())/2,(l1.A.y()+l1.B.y())/2,(l1.A.z()+l1.B.z())/2);
    Eigen::Vector3d g2=Eigen::Vector3d((l2.A.x()+l2.B.x())/2,(l2.A.y()+l2.B.y())/2,(l2.A.z()+l2.B.z())/2);

    double dist1,dist2, dist;
    dist1=PointLineDistance(g1,l2);
    dist2=PointLineDistance(g2,l1);
    dist=(dist1+dist2)/2;
    return dist;
}
double SLine_To_SLine_dist(Line L1,Line L2)
{
    Eigen::Vector3d a=L1.B-L1.A;
    Eigen::Vector3d b=L2.B-L2.A;
    Eigen::Vector3d c=L2.A-L1.A;

    double D=std::abs(c.dot(a.cross(b)))/(a.cross(b)).norm();
    return D;

}


double Angle(Eigen::Vector3d v1, Eigen::Vector3d v2)

{
    

    double s=v1.dot(v2);

    //H=||normal1||
    double t=v1.norm();
    // std::cout<<t<<std::endl;
    // L=||normal2||
    double u=v2.norm();
    // std::cout<<u<<std::endl;
    double an=s/(t*u);
    //double Ang=acos(an);

    double Ang=(180/M_PI  )*(acos(an));
    //std::cout<<"Angle is:"<<Ang<<"degree"<<std::endl;
    
    return Ang;
}
Eigen::Matrix3d vec_X(Eigen::Vector3d v)
{

    Eigen::Matrix3d Mat= Eigen::Matrix3d::Identity();
    Mat(0,0)=0;
    Mat(0,1)=-v.z();
    Mat(0,2)=v.y();
    Mat(1,0)=v.z();
    Mat(1,1)=0;
    Mat(1,2)=-v.x();
    Mat(2,0)=-v.y();
    Mat(2,1)=v.x();
    Mat(2,2)= 0;

    return Mat;
}
Eigen::Vector3d  Bisect_vector(Eigen::Vector3d v1, Eigen::Vector3d v2)
{ Eigen::Vector3d c;
    double d=v1.dot(v2);
    if(d<0)
        c=v1-v2;
    else
        c=v1+v2;
    return c/c.norm();


}
////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3d closest_point(Line l1, Line l2)
{   Eigen::Vector3d P;
    Eigen::Matrix3d M1,M2,M3,M4,M5,M;
    M1=vec_X(l1.d);
    M2=M1.transpose();
    M3=vec_X(l2.d);
    M4=M3.transpose();
    //M=((M2*M1)+(M4*M3) ).inverse();
    M5=((M2*M1)+(M4*M3));
    double d= M5.determinant();
    if((d/M5.norm())<1e-11)
        P=Eigen::Vector3d((l1.A.x()+l2.A.x())/2,(l1.A.y()+l2.A.y())/2,(l1.A.z()+l2.A.z())/2);
    else
    { M=M5.inverse();
        Eigen::Vector3d N1,N2,N;
        N1=(M2*M1)*l1.A;
        N2=(M4*M3)*l2.A;
        N=N1+N2;
        P=M*N;}
    return P;
}
////////////////////////////////////////////////
Eigen::Vector3d point_proj_bisect (Eigen::Vector3d p,Line l1, Line l2)
{
    Eigen::Vector3d pt=closest_point(l1,l2);
    Eigen::Vector3d v=Eigen::Vector3d(p-pt );
    Eigen::Vector3d vec=Bisect_vector(l1.d,l2.d);
    Eigen::Vector3d proj=pt+(v.dot(vec))*vec;
    //std::cout<<"point="<<pt<<std::endl;
    return proj;
}
/////////////////////
double overlap(Line l1, Line l2)
{double over=0;
    Eigen::Vector3d pt=closest_point(l1,l2);
    //std::cout<<"point="<<pt<<std::endl;
    Eigen::Vector3d vec=Bisect_vector(l1.d,l2.d);
    Eigen::Vector3d p1=l1.A;//point_proj_bisect(l1.A,l1,l2);
    Eigen::Vector3d p2=l1.B;//point_proj_bisect(l1.B,l1,l2);
    Eigen::Vector3d p3=l2.A;//point_proj_bisect(l2.A,l1,l2);
    Eigen::Vector3d p4=l2.B;//point_proj_bisect(l2.B,l1,l2);
    Eigen::Vector3d P1=p1-pt;
    Eigen::Vector3d P2=p2-pt;
    Eigen::Vector3d P3=p3-pt;
    Eigen::Vector3d P4=p4-pt;
    double c1=P1.dot(vec);
    double c2=P2.dot(vec);
    double s1=std::max(c1,c2)-std::min(c1,c2);
    double c3=P3.dot(vec);
    double c4=P4.dot(vec);

    double s2=std::max(c3,c4)-std::min(c3,c4);
    double s;
    if((l1.d.dot(l2.d)==1)&&((l1.length==l2.length)))
        s=l1.length;
    else if((l1.d.dot(l2.d)==1)&&((l1.length!=l2.length)))
    {

        Eigen::Vector3d g1=Eigen::Vector3d((l1.A.x()+l1.B.x())/2,(l1.A.y()+l1.B.y())/2,(l1.A.z()+l1.B.z())/2);
        Eigen::Vector3d g2=Eigen::Vector3d((l2.A.x()+l2.B.x())/2,(l2.A.y()+l2.B.y())/2,(l2.A.z()+l2.B.z())/2);

        double dist1,dist2, dist;
        dist1=PointLineDistance(g1,l2);
        dist2=PointLineDistance(g2,l1);
        dist=(dist1+dist2)/2;
        if(dist<1e-18)
        {if(std::max(c1,c2)<std::min(c3,c4))
                s=std::max(c1,c2)-std::min(c3,c4);
            else

                s=0;
        }

        else
            s=std::min(std::max(c1,c2),std::max(c3,c4))-std::max(std::min(c1,c2),std::min(c3,c4));
    }

    else
        s=std::min(std::max(c1,c2),std::max(c3,c4))-std::max(std::min(c1,c2),std::min(c3,c4));


    if(s<0)

        return 0;
    else
        return s/std::min(s1,s2);



}

