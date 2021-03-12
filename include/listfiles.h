#ifndef LISTFILES_H
#define LISTFILES_H


#include <iostream>
#include <math.h>
#include <fstream>
#include <string>

#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

//for search
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

//for RANSAC plane segment
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));

class List_Files
{
public:

    List_Files(){};

 

    vector<string> allfileList(string dir_path,bool dirfile);


    ~List_Files(){};
    
private:


};


// 0 for cosider the dir and 1 for only cosider the file
vector<string> List_Files::allfileList(string dir_path,bool dirfile)
{
    vector<string> sample_list;

    DIR *dir_ptr;

    dir_ptr=opendir(dir_path.c_str());  

    struct dirent *ent;

    chdir(dir_path.c_str());

    while((ent=readdir(dir_ptr))!=NULL)
    {
        if(strcmp(ent->d_name,".")==0||strcmp(ent->d_name,"..")==0)
        {
            continue;
        }
        struct stat st;  
        stat(ent->d_name,&st);
        if(S_ISDIR(st.st_mode)&&(!dirfile))
        {   
            sample_list.push_back(ent->d_name);

        }        

        if(S_ISREG(st.st_mode)&&dirfile)
        {
            sample_list.push_back(ent->d_name);
        }
    }    
    closedir(dir_ptr);
    chdir(".."); 


    sort(sample_list.begin(),sample_list.end());


    return sample_list;        
}



#endif