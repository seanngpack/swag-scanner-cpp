#include "Controller.h"

controller::Controller::Controller(camera::ICamera *camera,
                                   arduino::Arduino *arduino,
                                   model::Model *model,
                                   visual::Visualizer *viewer,
                                   file::FileHandler *file_handler) :
        camera(camera), model(model), arduino(arduino), viewer(viewer), file_handler(file_handler) {}


void controller::Controller::scan(int degs) {
    if (360 % degs != 0) {
        throw std::invalid_argument("Invalid input, scanning input must be a factor of 360");
    }
    int num_rotations = 360 / degs;

    const camera::ss_intrinsics *intrin = camera->get_intrinsics();
    std::cout << "starting scanning..." << std::endl;
    for (int i = 0; i < num_rotations; i++) {
        std::string name = std::to_string(i * degs);
        const uint16_t *depth_frame = camera->get_depth_frame();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = model->create_point_cloud(depth_frame, intrin);
        file_handler->save_cloud(cloud, name, CloudType::Type::RAW);
        arduino->rotate_table(degs);
    }
}

void controller::Controller::process_data() {
}

void controller::Controller::filter_clouds(std::string folder_path, CloudType::Type cloud_type) {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr>> cloud_vector;
    file_handler->load_clouds(cloud_vector, cloud_type, folder_path);
    for (int i = 0; i < cloud_vector.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr croppedCloud = model->crop_cloud(cloud_vector[i],
                                                                             -.15, .15,
                                                                             -100, .08,
                                                                             -100, .48);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud = model->voxel_grid_filter(croppedCloud);
        file_handler->save_cloud(filteredCloud, std::to_string(i), CloudType::Type::FILTERED);
    }
}

void controller::Controller::segment_clouds(std::string folder_path, CloudType::Type cloud_type) {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr>> cloud_vector;
    file_handler->load_clouds(cloud_vector, cloud_type, folder_path);
    for (int i = 0; i < cloud_vector.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedCloud = model->remove_plane(cloud_vector[i]);
        file_handler->save_cloud(segmentedCloud, std::to_string(i), CloudType::Type::SEGMENTED);
    }
}


void controller::Controller::register_all_clouds(std::string folder_path, CloudType::Type cloud_type) {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr>> cloud_vector;
    file_handler->load_clouds(cloud_vector, CloudType::Type::RAW, folder_path);
    Eigen::Matrix4f global_transform;
    pcl::PointCloud<pcl::PointXYZ>::Ptr alignedInitialCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // align second cloud to first cloud
    model->align_clouds(cloud_vector[1], cloud_vector[0], alignedInitialCloud, global_transform);
    *finalCloud = *cloud_vector[0] + *alignedInitialCloud;
    for (int i = 2; i < cloud_vector.size(); i++) {
        // move this elsewhere once we get started with ICP
        global_transform *= global_transform;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud_vector[i], *transformedCloud, global_transform);
    }
}

void controller::Controller::visualize_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    viewer->simpleVis(cloud);
}

controller::Controller::~Controller() {
    delete camera;
    delete arduino;
    delete model;
    delete viewer;
    delete file_handler;
}




