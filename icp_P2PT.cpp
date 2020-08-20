#include "validationlib.h"

int main(int argc, char** argv)
{
    if (argc < 3) {
        printf("Missing parameters\n");
        printf("Usage: icp_P2PT <target> <source>\n");

        return -1;
    }

    clock_t tempo;
    tempo = clock();
    
    PointCloud::Ptr source_cloud(new PointCloud);
    PointCloud::Ptr target_cloud(new PointCloud);
    PointCloud::Ptr icp_cloud(new PointCloud);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *target_cloud) == -1) {
        PCL_ERROR("Couldn't read target cloud\n");
        return -1;
    }
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *source_cloud) == -1) {
        PCL_ERROR("Couldn't read source cloud\n");
        return -1;
    }
 int iteration = atoi(argv[3]);

   // for(int iteration=1; iteration<iteration+1; iteration++){

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(source_cloud);
        icp.setInputTarget(target_cloud);
        icp.setMaximumIterations(iteration);
        icp.align(*icp_cloud);

        Eigen::Matrix4f rotation_matrix = icp.getFinalTransformation();
        double rms = computeCloudRMSE(target_cloud, icp_cloud, std::numeric_limits<double>::max());
        double elem1 = rotation_matrix(0, 0);
        double elem2 = rotation_matrix(1, 1);
        double elem3 = rotation_matrix(2, 2);
        double angle123 = (elem1 + elem2 + elem3 - 1) / 2.0;
        double rot_angle = (acos(angle123) * 180.0) / PI;

            std::cout << rotation_matrix << std::endl;
         std::cout << "RMSE"<< rms << std::endl;
         std::cout << "rotacao" <<  rot_angle << std::endl;
        std::cout << (clock() - tempo) / (double)CLOCKS_PER_SEC << std::endl;


        int sizePoints = 2;

        //Primeiro view
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Pos ICP"));
        viewer->setBackgroundColor(255,255,255);
        viewer->addPointCloud(icp_cloud,"cloud_in");
        viewer->addPointCloud(target_cloud,"cloud_out");

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255,"cloud_in");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "cloud_in");

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,255,0,"cloud_out");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "cloud_out");

        while(!viewer->wasStopped())
        {
            viewer->spinOnce();
            boost::this_thread::sleep (boost::posix_time::microseconds(100000));
        }

        //  return 0;
    }


