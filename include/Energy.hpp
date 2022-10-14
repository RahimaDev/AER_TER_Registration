#include "Transformation.hpp"
double Energy(std::vector<Line> ln1, std::vector<Line>ln2,Eigen::Matrix4d T,Eigen::Matrix3d R0,double th)

{
    int k=0;
    double sum=0;
    for(int l=0; l<ln1.size(); l++)
    { double h=(th*th);
        Line l0=tr_line(ln1[l],R0);
        
        Line l1=tr_line(l0,T);
        double S=0,S1=0;;

        for(int f=0; f<ln2.size(); f++)
        {


            Line l2=ln2[f];
            // Eigen::Vector3d d1=direct_vec(l1.A,l1.B);
            //Eigen::Vector3d d2=direct_vec(l2.A,l2.B);

            if(std::abs(l1.d.dot(l2.d))>0.95)
            {
                Eigen::Vector3d g1=Eigen::Vector3d((l1.A.x()+l1.B.x())/2,(l1.A.y()+l1.B.y())/2,(l1.A.z()+l1.B.z())/2);
                Eigen::Vector3d g2=Eigen::Vector3d((l2.A.x()+l2.B.x())/2,(l2.A.y()+l2.B.y())/2,(l2.A.z()+l2.B.z())/2);

                double dist1,dist2, dist;
                dist1=PointLineDistance(g1,l2);
                dist2=PointLineDistance(g2,l1);
                dist=(dist1+dist2)/2;


                double d=h-(dist*dist);

                double d1=std::max(0.0,d);

                if(d1>0)
                {


                    double over=  overlap( l1, l2);



                    S+=over*d1;;}}}
        S1=h-S;
        sum+=S1;


    }
    return sum;
}




