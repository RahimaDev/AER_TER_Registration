#include"match_data.hpp"
using namespace std;
//#define ep 0.0523599//3

//#define ep 0,174533 //10
/////////////////////////////////////////////////////////////////////////////////:
struct info_l
{
    Line l;
    double score;
    int  F;



};
struct Cluster
{
    std::vector<Line> Lines;
    Eigen::Vector3d clus_normal;
    string label;
};



Eigen::Vector3d Mean_dir(Cluster C)
{double dir_x=0;
    double dir_y=0;
    double dir_z=0;
    double s_w=0;
    Eigen::Vector3d w_dir(0,0,0);
    for(int i=0; i<C.Lines.size(); i++)
    { double w=C.Lines[i].length/C.Lines[0].length;
        dir_x+=w*C.Lines[i].d.x();
        dir_y+=w*C.Lines[i].d.y();
        dir_z+=w*C.Lines[i].d.z();
        s_w+=w;
    }
    //std::cout<<s_w<<" "<<"s_w"<<std::endl;

    w_dir.x()=dir_x/s_w;
    w_dir.y()=dir_y/s_w;
    w_dir.z()=dir_z/s_w;
    w_dir.normalize();
    //    std::cout<<"////////////////////////"<<std::endl;


    return w_dir;
}

///////////////////////////////////////////////////////////////////////////////////////////////////





int k_max(std::vector<Cluster> vec, Eigen::Vector3d d)
{

    int l=-1;
    double max=0;
    for(int i=0; i<vec.size(); i++)
    {
        double val=std::abs( Mean_dir(vec[i]).dot(d));
        if(val>max)
        {max=val;
            l=i;
        }
    }
    return l;
}












//////////////////////////////////////////////////////////////////////////////////////////////
std::vector<Cluster> Cluster_generation(std::vector<Line> pl )
{
    std::vector<Cluster> res;
    Cluster C1;
    C1.Lines.push_back(pl[0]);
    pl.erase(pl.begin());
    res.push_back(C1);

    //////////////////////////////////////

    for(int i=0; i<pl.size(); i++)
    {

        int k=k_max(res,pl[i].d);
        double val=std::abs( Mean_dir(res[k]).dot(pl[i].d));


        if(val>0.98)
        {

            if(res[k].Lines[0].d.dot(pl[i].d)<0)
            {

                Line L2;
                L2.A=pl[i].B;
                L2.B=pl[i].A;
                L2.d=direct_vec(L2.A,L2.B);
                L2.length= pl[i].length;
                L2.Id=pl[i].Id;



                res[k].Lines.push_back(L2);
            }
            else
                res[k].Lines.push_back(pl[i]);
        }
        else
        {


            Cluster C;

            C.Lines.push_back(pl[i]);

            res.push_back(C);
        }
    }
    std::cout<<"res="<<res.size()<<std::endl;

    //std::cout<<"/////////////////////////////////////////////////////"<<std::endl;
    return res;
}



