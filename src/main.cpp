#include <listfiles.h>

#include <iostream>
#include <fstream>
#include <string>


using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Select plane"));
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("viewer"));

 
struct callback_args{

    PointCloud::Ptr original_point;
    PointCloud::Ptr clicked_point_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};


/* callback function */
void pp_callback(const pcl::visualization::AreaPickingEvent& event,void* args)
{

    struct callback_args* data= (struct callback_args *) args;
    data->viewerPtr->removePointCloud("ROI_cloud");
    std::vector<int > indices;
    if (event.getPointsIndices(indices)==-1)
        return;

    for (int i=0;i<indices.size();i++)
    {
        data->clicked_point_3d->points.push_back(data->original_point->points.at(indices[i]));
       
    }

    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_point_3d,255,0,0);

    data->viewerPtr->addPointCloud(data->clicked_point_3d,red,"ROI_cloud");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10,"ROI_cloud");
 
}

  
/* Horizational Calibration for one pcd file*/ 
Eigen::Matrix4d HorizontalCalibration(string pcd_file_path)
{

    cout<<"current deal:"<<pcd_file_path<<endl;

    PointCloud::Ptr cloud(new PointCloud);

    if (pcl::io::loadPCDFile<PointT>(pcd_file_path,*cloud)==-1 )
    {
        PCL_ERROR ("Couldn't read the pcd file");
    }

    // cout<<cloud->size()<<endl;


    //for choice the point cloud
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer->addPointCloud<PointT>(cloud,rgb,"input_cloud");
    viewer->addCoordinateSystem();
    viewer->setCameraPosition(0,0,-2,0,-1,0);

    struct callback_args cb_args;
    PointCloud::Ptr clicked_points_3d (new PointCloud);
    cb_args.original_point=cloud;
    cb_args.clicked_point_3d=clicked_points_3d;
    cb_args.viewerPtr=pcl::visualization::PCLVisualizer::Ptr(viewer);
    viewer->registerAreaPickingCallback(pp_callback,(void*)&cb_args);


    viewer->spin();

    std::cout<<"Finish the point chose"<<std::endl;
    // cb_args.viewerPtr->removeAllPointClouds();
    viewer->removePointCloud("input_cloud");
    
    cout<<cb_args.clicked_point_3d->size()<<endl;

    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb1(cb_args.clicked_point_3d);
    
    /* RANSAC */
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliners (new pcl:: PointIndices);

    pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients(true);

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);
    
    seg.setInputCloud(cb_args.clicked_point_3d);
    seg.segment(*inliners,*coefficients);

    cout<<"plane parameters: "<<endl;
    cout<<coefficients->values[0]<<" "
        <<coefficients->values[1]<<" "
        <<coefficients->values[2]<<" "
        <<coefficients->values[3]<<endl;

    cout<<inliners->indices.size()<<endl;

    // extract the point according to the indices
    pcl::ExtractIndices<PointT> extract_indices;
    extract_indices.setIndices(boost::make_shared<const pcl::PointIndices>(*inliners));

    extract_indices.setInputCloud(cb_args.clicked_point_3d);
    PointCloud::Ptr output_plane (new PointCloud);
    extract_indices.filter(*output_plane);


    // for show the normal is visualization
    pcl::PointXYZ pointonplane(output_plane->points[0].x,output_plane->points[0].y,output_plane->points[0].z);
    pcl::PointXYZ porintonnorm(output_plane->points[0].x+2*coefficients->values[0],
                                output_plane->points[0].y+2*coefficients->values[1],
                                output_plane->points[0].z+2*coefficients->values[2]);
    
    pcl::PointXYZ pointonzero(0,0,0);
    pcl::PointXYZ pointawayzero(0,-1,0);

    // calculate the Rotate matrix
    PointCloud::Ptr transform_cloud (new PointCloud );

    Eigen::Vector3d vplane(0,coefficients->values[1],coefficients->values[2]);
    Eigen::Vector3d vy(0,-1,0);
    Eigen::Matrix3d R_vector;
    R_vector=Eigen::Quaterniond::FromTwoVectors(vplane,vy).toRotationMatrix();



    Eigen::Matrix4d transform_R=Eigen::Matrix4d::Identity();
    transform_R.topLeftCorner(3,3)=R_vector;
    pcl::transformPointCloud(*cloud,*transform_cloud,transform_R);


    cout<<"Rotation Matrix: "<<endl;
    cout<<transform_R<<endl;

    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("viewer1"));
 
    // viewer->addPointCloud<PointT>(cb_args.clicked_point_3d,rgb1,"cloud");

    // pcl::visualization::PointCloudColorHandlerCustom<PointT> output_handler(output_plane,255,0,0);
    // viewer->addPointCloud<PointT>(output_plane,output_handler,"estimate plane");

    // viewer->addLine<pcl::PointXYZ>(pointonplane,porintonnorm,255,0,0,"line");
    // viewer->addLine<pcl::PointXYZ>(pointonzero,pointawayzero,0,255,0,"line2");
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> transform_cloud_handler(transform_cloud);
    viewer1->addPointCloud<PointT>(transform_cloud,transform_cloud_handler,"transform plane");
    viewer1->addCoordinateSystem();
    viewer1->setCameraPosition(0,0,-2,0,-1,0);

    viewer1->spin();

    viewer1->removePointCloud("transform plane");

    return transform_R;

}


int main()
{

    // pcd file dir (contain the pcd(point cloud) file)
    string pcd_dir_path="./pcdFiles/";

    // resume file used to resume the procdure from last time
    string resume_file_path="./calibFiles/resume.txt";

    // the path used to save the calibration plane
    string RotationMatrix_dir_path="./calibFiles/Rotation_matrix/";



    List_Files list_files;

    fstream resumefile;



    string resume_id;

    resumefile.open(resume_file_path.c_str(),ios::out|ios::in);
    
    resumefile>>resume_id;

    cout<<"resume_id: "<<resume_id<<endl;

    vector<string>  pcd_list= list_files.allfileList(pcd_dir_path.c_str(),true);


    vector<string>::iterator it=pcd_list.begin();
    for(;it!=pcd_list.end();++it)
    {
        if(*it==resume_id) break;

    }

    for(;it!=pcd_list.end();++it)
    {
        if (*it=="resume.txt") continue;

        resumefile.seekg(0,ios::beg);
        resumefile<<*it<<endl;

        Eigen::Matrix4d R_matrix= HorizontalCalibration(pcd_dir_path+*it);

        string R_matrixfile_path=RotationMatrix_dir_path+it->substr(0,8)+".txt";

        // if file not exist creat it and put the rotate matrix to txt
        fstream R_matrixfile_old;
        R_matrixfile_old.open(R_matrixfile_path.c_str(),ios::out);
        if (! R_matrixfile_old)
        {
            cout<<"the file not exists"<<endl;

            ofstream R_matrixfile_new(R_matrixfile_path.c_str());
            if (R_matrixfile_new)
            {
                R_matrixfile_new<<R_matrix<<endl;

                R_matrixfile_new.close();
            }
        }
        else
        {
            R_matrixfile_old<<R_matrix<<endl;

            R_matrixfile_old.close();           
        }

        cout<<"-------------------"<<endl;

    }


    resumefile.close();
    
 
    
    return 0;
}


