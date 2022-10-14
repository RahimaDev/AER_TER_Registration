

#include <pcl/io/ply_io.h>

#include <pcl/point_types.h>









#include <pcl/common/transforms.h>
#include<pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/pcl_painter2D.h>
#include <pcl/visualization/pcl_visualizer.h>



#include"clustering.hpp"

double angles(Eigen::Matrix3d R)
{

    double thetaX ;
    double thetaY ;
    double thetaZ;
    if ( R(0,2) < +1)
    {
        if ( R(0,2) > -1)
        {
            thetaY = asin (R(0,2) ) ;
            thetaX = atan2 (-R(1,2) , R(2,2) ) ;
            thetaZ = atan2 (-R(0,1) , R(0,0) ) ;
        }
        else
        {
            thetaY = -M_PI / 2 ;
            thetaX = -atan2( R(1,0) , R(1,1) ) ;
            thetaZ = 0;
        }
    }
    else
    {
        thetaY=+M_PI / 2 ;
        thetaX=atan2 ( R(1,0) , R(1,1) ) ;
        thetaZ=0;
    }

    double M=(thetaX+thetaY+thetaZ)/3;
    //std::cout<<"mean="<<M<<std::endl;
    return M;
}




Eigen::Vector3d V_proj(Eigen::Vector3d U1, Eigen::Vector3d V1)
{
    double  v2_ls = U1.norm()*U1.norm();
    //std::cout<<"the values="<<v2_ls<<std::endl;
    Eigen::Vector3d Vec=V1-(U1*(V1.dot(U1)/v2_ls));
    Eigen::Vector3d Vec1=Vec.normalized();
    return Vec1;

}

Eigen::Matrix3d Rotation(Eigen::Vector3d u0,Eigen::Vector3d u1,Eigen::Vector3d v0,Eigen::Vector3d v1)
{
    Eigen::Vector3d v00=V_proj(u0,v0);

    //std::cout<<"prod="<<u0.dot(v0_0)<<std::endl;
    Eigen::Vector3d s=u0.cross(v00);
    Eigen::Vector3d s0=s.normalized();
    Eigen::Vector3d v11=V_proj(u1,v1);

    Eigen::Vector3d q=u1.cross(v11);
    //std::cout<<"prod2="<<u1.dot(v1_1)<<std::endl;
    Eigen::Vector3d q0=q.normalized();
    Eigen::Matrix3d M1;
    M1.col(0)=u0;
    M1.col(1)=v00;
    M1.col(2)=s0;
    Eigen::Matrix3d M2;
    M2.col(0)=u1;
    M2.col(1)=v11;
    M2.col(2)=q0;
    Eigen::Matrix3d R=M2*M1.inverse();
    return R;
}
























Line tr_line(Line l,  Eigen::Matrix4d  T)
{

    Line f;
    Eigen::Vector4d A1=Eigen::Vector4d(l.A.x(), l.A.y(), l.A.z(),1);
    Eigen::Vector4d B1=Eigen::Vector4d(l.B.x(), l.B.y(), l.B.z(),1);
    Eigen::Vector4d C1=T.matrix()*A1;
    Eigen::Vector4d C2=T.matrix()*B1;
    Eigen::Vector3d A2=Eigen::Vector3d(C1.x(), C1.y(), C1.z());
    Eigen::Vector3d B2=Eigen::Vector3d(C2.x(), C2.y(), C2.z());
    f.A=A2;
    f.B=B2;
    f.d= direct_vec(A2,B2);
    pcl::PointXYZ p1=pcl::PointXYZ(A2.x(), A2.y(), A2.z());
    pcl::PointXYZ p2=pcl::PointXYZ(B2.x(), B2.y(), B2.z());
    f.length=pcl::geometry::distance(p1,p2);
    f.Id=l.Id;
    return f;
}

Line tr_line(Line l, Eigen::Matrix3d R)
{

    Line f;


    Eigen::Vector3d A2=R*l.A;
    Eigen::Vector3d B2=R*l.B;
    f.A=A2;
    f.B=B2;
    f.d= direct_vec(A2,B2);
    f.Id=l.Id;
    pcl::PointXYZ p1=pcl::PointXYZ(A2.x(), A2.y(),A2.z());
    pcl::PointXYZ p2=pcl::PointXYZ(B2.x(), B2.y(), B2.z());
    f.length=pcl::geometry::distance(p1,p2);
    return f;

}
Eigen::Matrix4d scale_tr(std::vector< Eigen::Vector3d> src,std::vector<  Eigen::Vector3d> dst,std::vector<  Eigen::Vector3d> dir){
    assert(src.size()==dst.size() && src.size()==dir.size());
    Eigen::Matrix<double,4,4> C; C.setZero();
    Eigen::Matrix<double,4,1> d; d.setZero();



    for(int i=0;i<src.size();++i){
        C.block<3,3>(0,0)+=vec_X(dir[i])*vec_X(dir[i]);
        
        C.block<3,1>(0,3)+=vec_X(dir[i])*vec_X(dir[i])*src[i];
        C(3,3)-=((dir[i]).cross(src[i])).squaredNorm();
        d.head(3)+=vec_X(dir[i])*vec_X(dir[i])*dst[i];
        d(3)-=(dir[i].cross(src[i])).dot(dir[i].cross(dst[i]));
        
    }

    C.block<1,3>(3,0)=C.block<3,1>(0,3).transpose();



    //C.block<3,3>(3,3)=-C.block<3,3>(0,0).transpose();
    Eigen::Matrix<double,4,1> x =C.inverse()*d;//C.colPivHouseholderQr().solve(d);
    //std::cout<<"X="<<x<<std::endl;
    Eigen::Matrix4d T =   Eigen::Matrix4d::Identity();
    T(0,0)=x(3);
    T(1,1)=x(3);
    T(2,2)=x(3);
    T(0,3)=x(0);
    T(1,3)=x(1);
    T(2,3)=x(2);
    //std::cout<<T<<std::endl;
    return T;
}






