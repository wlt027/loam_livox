//
// Created by wlt-zh on 9/14/18.
//

#include "RosVisualization.h"
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>


namespace vis {

    static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R) {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / M_PI * 180.0;
    }

    RosVisualization::RosVisualization()
    :required_stop_(false), is_finished_(false),_laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZI>()),
     ramp(COLORARGB(255,0,0,180),COLORARGB(255,180,0,0),4),
     _laserCloudSurroundDS(new pcl::PointCloud<pcl::PointXYZI>()){

        m_bShowIntensity = true;
        m_bShowBigPointSize = false;
        m_bOutput = false;
        bAlignRef = false;

        curT = 0;

        pangolin_thread_ = std::make_shared<std::thread>(std::bind(&RosVisualization::run, this));

        image_size_ = cv::Size(752,480);

        nScanCount = 0;

        strOut = "";

        initQ = Eigen::Quaterniond(1,0,0,0);

        nSaveFreq = 1;

        refR.setIdentity();
        refP = Eigen::Vector3d(0,0,0);
//        P_B_C = Eigen::Vector3d(0,0,0);
//        R_B_C.setIdentity();
    }

    bool RosVisualization::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode) {

        // subscribe to laser odometry topics
        _subLaserCloudSurround = node.subscribe<sensor_msgs::PointCloud2>
                ("/laser_cloud_surround", 1, &RosVisualization::laserCloudSurroundHandler, this);

        _subLaserCloudFullRes = node.subscribe<sensor_msgs::PointCloud2>
                ("/velodyne_cloud_registered", 2, &RosVisualization::laserCloudFullResHandler, this);

        _sub_track_img_compress = node.subscribe<sensor_msgs::CompressedImage>
                ("/usb_cam/image_raw/compressed",100,&RosVisualization::trackedImageHandler,this);

        _sub_track_img = node.subscribe<sensor_msgs::Image>
                ("/vins_node/image_track",1000,&RosVisualization::trackedImageHandler,this);

        // subscribe to laser odometry and mapping odometry topics
//        _subLaserOdometry = node.subscribe<nav_msgs::Odometry>
//                ("/aft_mapped_to_init_high_frec", 5, &RosVisualization::laserOdometryHandler, this);

        _subRefOdom0 = node.subscribe<geometry_msgs::PointStamped>
                ("/leica/position", 5, &RosVisualization::refVioPose0Handler, this);

        _subRefOdom1 = node.subscribe<geometry_msgs::TransformStamped>
                ("/vicon/firefly_sbx/firefly_sbx", 5, &RosVisualization::refVioPose1Handler, this);

        _subRefOdom2 = node.subscribe<geometry_msgs::PoseStamped>
                ("/Robot_7/pose", 5, &RosVisualization::reflioPoseHandler, this);

        _subOdom = node.subscribe<nav_msgs::Odometry>
                ("/vins_node/odometry", 5, &RosVisualization::vioPoseHandler, this);

        _sub_visual_map = node.subscribe<sensor_msgs::PointCloud>
                ("/vins_node/point_cloud",1,&RosVisualization::visualMapHandler, this);

        _subOdomAftMapped = node.subscribe<nav_msgs::Odometry>
                ("/aft_mapped_to_init", 5, &RosVisualization::odomAftMappedHandler, this);

        _sub_init_rot = node.subscribe<nav_msgs::Odometry>
                ("/init_rot", 5, &RosVisualization::initRotHandler, this);

        _pubRestart = node.advertise<std_msgs::Bool>("/restart",5);

        _pubSave = node.advertise<std_msgs::Bool>("/save",5);

        node.getParam("out_path", strOut);
        printf("output:%s\n",strOut.data());

        node.param<int>("save_freq", nSaveFreq, 1);
//        printf("nSaveFreq:%d\n",nSaveFreq);
        node.param<int>("width", width, 1);
        node.param<int>("height", height, 1);

        image_size_ = cv::Size(width,height);

        printf("w,h:%d,%d\n",image_size_.width,image_size_.height);

        return true;
    }

    void RosVisualization::pubRestartMessage()
    {
        // send restart message
        ROS_INFO("pub restart message from viewer");
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        _pubRestart.publish(restart_flag);

        clearState();
    }

    void RosVisualization::pubSaveMessage()
    {
        // send save message
        ROS_INFO("pub save message from viewer");
        std_msgs::Bool save_flag;
        save_flag.data = true;
        _pubSave.publish(save_flag);
        
    }

    void RosVisualization::setStop() {
        std::lock_guard<std::mutex> lock(mutex_stop_);
        required_stop_ = true;
    }

    bool RosVisualization::isRequiredStop() {
        std::lock_guard<std::mutex> lock(mutex_stop_);
        return required_stop_;
    }

    bool RosVisualization::waitForFinish() {
        if (!isRequiredStop())
            setStop();

        if (pangolin_thread_->joinable())
            pangolin_thread_->join();

        return true;
    }

    void RosVisualization::clearState()
    {
        nScanCount = 0;
        mutex_pose_.lock();
        mTraj.clear();
        modoTraj.clear();
        mRefTraj0.clear();
        mRefTraj1.clear();
        mRefTraj2.clear();
        mutex_pose_.unlock();
        mutex_cur_pose_.lock();
        curPQ.P = Eigen::Vector3d(0,0,0);
        curPQ.Q = Eigen::Quaterniond(1,0,0,0);
        curT = 0;
        mutex_cur_pose_.unlock();

        mutex_laser_cloud_.lock();
        _laserCloudFullRes->clear();
        mutex_laser_cloud_.unlock();

        mutex_laser_cloud_surround_.lock();
        _laserCloudSurroundDS->clear();
        mutex_laser_cloud_surround_.unlock();

        mutex_image_.lock();
        cv::Mat image = cv::Mat::zeros(image_size_, CV_8UC3);
        image.copyTo(image_);
        mutex_image_.unlock();
    }

    void RosVisualization::run() {
        const string win_name = "LIVE Viewer";
        float mViewpointX = -5;
        float mViewpointY = 0;
        float mViewpointZ = 10;
        float mViewpointF = 500;
        float VISUALLOOKATX = 0;
        float VISUALLOOKATY = 0;
        float VISUALLOOKATZ = -1;

        int w, h;
        w = 640;
        h = 360;

        pangolin::CreateWindowAndBind(win_name, 2 * w, 2 * h); //create a display window
        glEnable(GL_DEPTH_TEST); //launch depth test
        glEnable(GL_BLEND);      //use blend function
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); //set blend alpha value

        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(2 * w, 2 * h, 500, 500, w, h, 0.1, 2000),
//                pangolin::ModelViewLookAt(0, 5, -5, 0, 0, 0, 0, 1, 1)
                pangolin::ModelViewLookAt(-5, 0, 5, 0, 0, 0, 1, 0, 1)
        );

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View &d_cam = pangolin::Display("camera")
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -w / (float) h)
                .SetHandler(new pangolin::Handler3D(s_cam));

        int width = image_size_.width;
        int height = image_size_.height;
        pangolin::View &d_video = pangolin::Display("imgVideo")
                .SetAspect(width / (float) height);

        pangolin::GlTexture texVideo(width,height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

        pangolin::CreateDisplay()
                //.SetBounds(0.0, 0.3, pangolin::Attach::Pix(175), 1.0)
                .SetBounds(0.0, 1.0, 0.8, 1.0)
                .SetLayout(pangolin::LayoutEqual)
                .AddDisplay(d_video);

        pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175)); //new button and menu
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
        pangolin::Var<bool> menuShowPath("menu.Show Path", true, true);
        pangolin::Var<bool> menuShowGTPath("menu.Show GT path", true, true);
        pangolin::Var<bool> menuViewSubmap("menu.Show Submap", true, true);
        pangolin::Var<bool> menuViewScan("menu.Show Scan", true, true);
        pangolin::Var<bool> menuViewIntensity("menu.Show Intenstiy",true,true);
        pangolin::Var<bool> menuViewImage("menu.Show Image", true, true);
        pangolin::Var<bool> menuViewVismap("menu.Show Visual Map", true, true);
        pangolin::Var<bool> menuViewBigPointSize("menu.BPS",false,true);
        pangolin::Var<bool> buttonViewFPS("menu.FPS", false, false);
        pangolin::Var<bool> buttonRestart("menu.Restart", false, false);
        pangolin::Var<bool> buttonSave("menu.Save", false, false);

        pangolin::OpenGlMatrix Twi;
        Twi.SetIdentity();
        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        while (!pangolin::ShouldQuit() && !isRequiredStop()) {

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            if (pangolin::Pushed(buttonViewFPS)) {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(-5, 0, 5, 0, 0, 0, 1, 0, 1));
            }

            if (pangolin::Pushed(buttonRestart)) {
                pubRestartMessage();
            }
            if(pangolin::Pushed(buttonSave)){
                pubSaveMessage();
            }

            getCurrentAxisPose(Twi);
            if (menuFollowCamera)
                s_cam.Follow(Twi);

            d_cam.Activate(s_cam);
            glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

            // draw origin coordinates
            pangolin::glDrawAxis(0.1);

            // draw current axis
            drawAxis(Twi);

            if(menuViewIntensity)
            {
                m_bShowIntensity = true;
            } else{
                m_bShowIntensity = false;
            }
            if(menuViewBigPointSize)
            {
                m_bShowBigPointSize = true;
            } else{
                m_bShowBigPointSize = false;
            }
            if (menuViewSubmap) {
                drawSubMapPoints();
            }

            if (menuViewScan) {
                drawCurrentScan();
            }
            if (menuShowPath) {
                drawTrajectory();
            }
            if(menuShowGTPath) {
                drawRefTrajectory();
            }

            if(menuViewVismap) {
                drawVisualMap();
            }
            if (menuViewImage && !image_.empty()) {
                cv::Mat image;
                mutex_image_.lock();
                image = image_;
                mutex_image_.unlock();

                drawCurrentImage(texVideo, image);
                //display the image
                d_video.Activate();
                glColor3f(1.0, 1.0, 1.0);
                texVideo.RenderToViewportFlipY();
            }

            pangolin::FinishFrame();
        }

        pangolin::DestroyWindow(win_name);
    }

    void
    RosVisualization::drawSubMapPoints() {
         if (_laserCloudSurroundDS && _laserCloudSurroundDS->size() > 0) {
             u32 beginColor = ramp.GetBeginColor();
             u32 endColor = ramp.GetEndColor();

            if(m_bShowBigPointSize)
            {
                glPointSize(2);
            }
            else
            {
                glPointSize(1);
            }
            glBegin(GL_POINTS);
            for (int i = 0; i < _laserCloudSurroundDS->size(); i++) {
                pcl::PointXYZI pt = (*_laserCloudSurroundDS)[i];

                if (pt.x == 0 && pt.y == 0 && pt.z == 0) {
                    continue;
                }

                float height = pt.z;
                float minZ = -1;
                float maxZ = 19;
                float fStep = 2;
                u8 a, r, g, b;

                if (height < minZ) {
                    r = ((beginColor >> 16) & 0xff);
                    g = ((beginColor >> 8) & 0xff);
                    b = (beginColor & 0xff);
                } else if (height > maxZ) {
                    r = ((endColor >> 16) & 0xff);
                    g = ((endColor >> 8) & 0xff);
                    b = (endColor & 0xff);
                } else {
                    int selStep = (int) ((height - minZ) / fStep);

                    float tempValue = height - minZ - selStep * fStep;
                    float scale = tempValue / fStep;
                    ramp.GetColor4ub(scale, a, r, g, b, selStep + 1);
                }

                if(m_bShowIntensity)
                {
//                    float inten = pt.intensity > 50.f ? 1.f : (float)pt.intensity / 50.f;
//                    glColor4f(1.0, 1.0, 1.0,inten);
//                    float inten = pt.intensity > 50.f ? 255 : pt.intensity;
                    glColor4ub(r, g, b,255);
                } else
                {
//                    glColor3f(1.0, 1.0, 1.0);
                    glColor3ub(r, g, b);
                }

                glVertex3f(pt.x, pt.y, pt.z);
            }
            glEnd();
        }
    }

    void
    RosVisualization::drawCurrentScan() {

        if (_laserCloudFullRes && _laserCloudFullRes->size() > 0) {
            glPointSize(1);
//            glColor3f(0.0, 1.0, 0.0);
            glColor3f(1.0, 1.0, 1.0);
            glBegin(GL_POINTS);
            for (int i = 0; i < _laserCloudFullRes->size(); i++) {
                pcl::PointXYZI pt = (*_laserCloudFullRes)[i];

                if (pt.x == 0 && pt.y == 0 && pt.z == 0) {
                    continue;
                }
                if (pcl_isnan(pt.x) || pcl_isnan(pt.y) || pcl_isnan(pt.z)) {
                    continue;
                }

                glVertex3f(pt.x, pt.y, pt.z);
                //cout<<pt.x<<","<<pt.y<<","<<pt.z<<endl;
            }
            glEnd();
        }
    }

    void
    RosVisualization::drawAxis(pangolin::OpenGlMatrix &Twi) {
        const float &w = 0.08f; //mCameraSize;
        const float h = w;
        const float z = w;

        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twi.m);
#else
        glMultMatrixd(Twi.m);
#endif

        glLineWidth(2);  //set line width
        glBegin(GL_LINES);           //draw axis
        // axis z
        glColor3f(0.0f, 0.0f, 1.0f);   //blue
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, z);
        // axis x
        glColor3f(1.0f, 0.0f, 0.0f);   //red
        glVertex3f(0, 0, 0);
        glVertex3f(w, 0, 0);
        // axis y
        glColor3f(0.0f, 1.0f, 0.0f);   //green
        glVertex3f(0, 0, 0);
        glVertex3f(0, h, 0);

        glEnd();
        glPopMatrix();
    }


    void
    RosVisualization::getCurrentAxisPose(pangolin::OpenGlMatrix &M) {

        if (curT > 0) {
            Eigen::Vector3d P = curPQ.P;
            Eigen::Matrix3d R = curPQ.Q.toRotationMatrix();

            M.m[0] = R(0, 0);
            M.m[1] = R(1, 0);
            M.m[2] = R(2, 0);
            M.m[3] = 0.0;

            M.m[4] = R(0, 1);
            M.m[5] = R(1, 1);
            M.m[6] = R(2, 1);
            M.m[7] = 0.0;

            M.m[8] = R(0, 2);
            M.m[9] = R(1, 2);
            M.m[10] = R(2, 2);
            M.m[11] = 0.0;


            M.m[12] = P.x();
            M.m[13] = P.y();
            M.m[14] = P.z();
            M.m[15] = 1.0;

        } else
            M.SetIdentity();
    }

    void
    RosVisualization::drawCurrentImage(pangolin::GlTexture &gl_texture, cv::Mat &image) {
        if (image.empty())
            return;

        if (image.type() == CV_8UC1)
            cv::cvtColor(image, image, CV_GRAY2RGB);
        gl_texture.Upload(image.data, GL_BGR, GL_UNSIGNED_BYTE);
    }


    void
    RosVisualization::drawTrajectory() {
        if (mTraj.size() > 0) {

            Eigen::Vector3d tmp_p;
            // red
            glColor3f(1.0f, 0.0f, 0.0f);
            glLineWidth(1);
            glBegin(GL_LINE_STRIP);

            for (int i = 0; i < mTraj.size(); ++i) {
                traj_point prePose = mTraj[i].second;
                tmp_p = prePose.P;
                glVertex3f((float) tmp_p(0), (float) tmp_p(1), (float) tmp_p(2));
            }
            glEnd();
        }

        if (modoTraj.size() > 0) {

            Eigen::Vector3d tmp_p;
            // red
            glColor3f(0.0f, 0.0f, 1.0f);
            glLineWidth(1);
            glBegin(GL_LINE_STRIP);

            for (int i = 0; i < modoTraj.size(); ++i) {
                traj_point prePose = modoTraj[i].second;
                tmp_p = prePose.P;
                glVertex3f((float) tmp_p(0), (float) tmp_p(1), (float) tmp_p(2));
            }
            glEnd();
        }
    }

    void
    RosVisualization::initRotHandler(const nav_msgs::Odometry::ConstPtr& Odo)
    {
        ROS_INFO("get init rot");

        if(Odo)
        {
            initQ.x() = Odo->pose.pose.orientation.x;
            initQ.y() = Odo->pose.pose.orientation.y;
            initQ.z() = Odo->pose.pose.orientation.z;
            initQ.w() = Odo->pose.pose.orientation.w;
        }
    }

    void
    RosVisualization::drawRefTrajectory()
    {
        if (mRefTraj0.size() > 0) {

            Eigen::Vector3d tmp_p;
            // red
            glColor3f(0.0f, 1.0f, 0.0f);
            glLineWidth(1);
            glBegin(GL_LINE_STRIP);

            for (int i = 0; i < mRefTraj0.size(); ++i) {
                traj_point prePose = mRefTraj0[i].second;
                tmp_p = prePose.P;

                tmp_p = refR*tmp_p + refP;
                glVertex3f((float) tmp_p(0), (float) tmp_p(1), (float) tmp_p(2));
            }
            glEnd();
        }

        if (mRefTraj1.size() > 0) {

            Eigen::Vector3d tmp_p;
            // red
            glColor3f(0.0f, 1.0f, 0.0f);
            glLineWidth(1);
            glBegin(GL_LINE_STRIP);

            for (int i = 0; i < mRefTraj1.size(); ++i) {
                traj_point prePose = mRefTraj1[i].second;
                tmp_p = prePose.P;
                tmp_p = refR*tmp_p + refP;
                glVertex3f((float) tmp_p(0), (float) tmp_p(1), (float) tmp_p(2));
            }
            glEnd();
        }

        if (mRefTraj2.size() > 0) {

            Eigen::Vector3d tmp_p;
            // red
            glColor3f(0.0f, 1.0f, 0.0f);
            glLineWidth(1);
            glBegin(GL_LINE_STRIP);

            for (int i = 0; i < mRefTraj2.size(); ++i) {
                traj_point prePose = mRefTraj2[i].second;
                tmp_p = prePose.P;
                tmp_p = refR*tmp_p + refP;
                glVertex3f((float) tmp_p(0), (float) tmp_p(1), (float) tmp_p(2));
            }
            glEnd();
        }
    }


    void RosVisualization::drawVisualMap()
    {
        if (m_visualMap.size() > 0) {
            glPointSize(2);
            glColor3f(1.0, 1.0, 0.0);
            glBegin(GL_POINTS);
            for (int i = 0; i < m_visualMap.size(); i++) {
                cv::Point3f pt = (m_visualMap)[i];

                glVertex3f(pt.x, pt.y, pt.z);
                //cout<<pt.x<<","<<pt.y<<","<<pt.z<<endl;
            }
            glEnd();
        }
    }

    void
    RosVisualization::laserCloudSurroundHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudSurroundDSMsg) {
        //ROS_INFO("get laser surround message success!");

        if(laserCloudSurroundDSMsg )
        {
            mutex_laser_cloud_surround_.lock();

            _laserCloudSurroundDS->clear();
            pcl::fromROSMsg(*laserCloudSurroundDSMsg, *_laserCloudSurroundDS);

            // transform laser cloud
            for(int i = 0; i< _laserCloudSurroundDS->size();i++)
            {
                pcl::PointXYZI& pt = _laserCloudSurroundDS->points[i];
                Eigen::Vector3d pt_tmp(pt.x,pt.y,pt.z);
                Eigen::Vector3d pt_w = initQ*pt_tmp;
                pt.x = pt_w(0);
                pt.y = pt_w(1);
                pt.z = pt_w(2);
            }

            mutex_laser_cloud_surround_.unlock();
        }
    }


    void RosVisualization::laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullResMsg) {
        //ROS_INFO("get 1 message success!");

        if(laserCloudFullResMsg )
        {
            mutex_laser_cloud_.lock();

            _laserCloudFullRes->clear();

            pcl::fromROSMsg(*laserCloudFullResMsg,*_laserCloudFullRes);

            // transform laser cloud
            for(int i = 0; i< _laserCloudFullRes->size();i++)
            {
                pcl::PointXYZI& pt = _laserCloudFullRes->points[i];
                Eigen::Vector3d pt_tmp(pt.x,pt.y,pt.z);
                Eigen::Vector3d pt_w = initQ*pt_tmp;
                pt.x = pt_w(0);
                pt.y = pt_w(1);
                pt.z = pt_w(2);
            }

            if(!strOut.empty() && nScanCount % nSaveFreq == 0)
            {
                {
                    std::string str = strOut + "/map.txt";

                    if(!m_bOutput)
                    {
                        m_bOutput = true;
                        FILE* pFile = fopen(str.data(),"w");

                        if(pFile)
                        {
                            for(int i = 0; i< _laserCloudFullRes->size();i++)
                            {
                                pcl::PointXYZI& pt = (*_laserCloudFullRes)[i];

                                fprintf(pFile,"%f,%f,%f,%f\n",pt.x,pt.y,pt.z,pt.intensity);
                            }
                            fclose(pFile);
                        }

                    }
                    else
                    {
                        FILE* pFile = fopen(str.data(),"at");

                        if(pFile)
                        {
                            for(int i = 0; i< _laserCloudFullRes->size();i++)
                            {
                                pcl::PointXYZI& pt = (*_laserCloudFullRes)[i];

                                fprintf(pFile,"%f,%f,%f,%f\n",pt.x,pt.y,pt.z,pt.intensity);
                            }
                            fclose(pFile);
                        }
                    }
                }
            }

            nScanCount++;
            mutex_laser_cloud_.unlock();

            //ROS_INFO("laser count: %d",_laserCloudFullRes->size());
        }
    }

    void RosVisualization::trackedImageHandler(const sensor_msgs::CompressedImage::ConstPtr& img_msg)
    {
        cv_bridge::CvImageConstPtr ptr;
        ptr = cv_bridge::toCvCopy(img_msg);

        cv::Mat show_img = ptr->image;

        if (!show_img.empty()) {
            mutex_image_.lock();
            show_img.copyTo(image_);
            mutex_image_.unlock();
        }
    }

    void RosVisualization::laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry) {

//        if (laserOdometry) {
//            mutex_vio_pose_.lock();
//
//            double tmpTm;
//            Eigen::Vector3d tmpP;
//            Eigen::Matrix3d tmpR;
//            Eigen::Quaterniond tmpQ;
//
//            tmpTm = laserOdometry->header.stamp.toSec();
//
//            tmpP(0) = laserOdometry->pose.pose.position.x;
//            tmpP(1) = laserOdometry->pose.pose.position.y;
//            tmpP(2) = laserOdometry->pose.pose.position.z;
//
//            tmpQ.x() = laserOdometry->pose.pose.orientation.x;
//            tmpQ.y() = laserOdometry->pose.pose.orientation.y;
//            tmpQ.z() = laserOdometry->pose.pose.orientation.z;
//            tmpQ.w() = laserOdometry->pose.pose.orientation.w;
//
//            tmpR = tmpQ.toRotationMatrix();
//            traj_point pq;
//            pq.P = tmpP;
//            pq.Q = tmpR;
//
//            mVioTraj.emplace_back(std::make_pair(tmpTm, pq));
//
//            mutex_vio_pose_.unlock();
//
//            if (mTraj.size() == 0) {
//                mutex_cur_pose_.lock();
//                curPQ.P = tmpP;
//                curPQ.Q = tmpQ;
//                curT = tmpTm;
//                mutex_cur_pose_.unlock();
//            }
//        }
    }

    void RosVisualization::odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr &odomAftMapped) {

//        ROS_INFO("get 2 message success!");

        if (odomAftMapped ) {
            geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;

            Eigen::Vector3d p(odomAftMapped->pose.pose.position.x,
                              odomAftMapped->pose.pose.position.y,
                              odomAftMapped->pose.pose.position.z);
            Eigen::Quaterniond q;
            q.w() = geoQuat.w;
            q.x() = geoQuat.x;
            q.y() = geoQuat.y;
            q.z() = geoQuat.z;

            p = initQ*p;
            q = initQ*q;

            Eigen::Vector3d ypr = R2ypr(q.toRotationMatrix());

            double tm = odomAftMapped->header.stamp.toSec();

            mutex_pose_.lock();
            traj_point pq;
            pq.P = p;
            pq.Q = q;

            mTraj.emplace_back(std::make_pair(tm, pq));
            mutex_pose_.unlock();

            ROS_INFO("PQ:%lf,%lf %lf %lf,%lf %lf %lf",tm,p(0),p(1),p(2),ypr(0),ypr(1),ypr(2));

            if(!bAlignRef)
            {
                // calculate
                mutex_ref_pose_.lock();

                if(mRefTraj2.size() > 1)
                {
                    for (int i = 1; i < mRefTraj2.size(); ++i) {
                        if (mRefTraj2[i].first > tm) {
                            double tmpPreTime = mRefTraj2[i - 1].first;
                            double tmpAftTime = mRefTraj2[i].first;

                            double scale = (tm - tmpPreTime) / (tmpAftTime - tmpPreTime);

                            Eigen::Quaterniond tmpQ = mRefTraj2[i - 1].second.Q.slerp(scale, mRefTraj2[i].second.Q);
                            tmpQ = q*(tmpQ.inverse());
                            refR = tmpQ.toRotationMatrix();

                            Eigen::Vector3d tmpP = mRefTraj2[i - 1].second.P + scale * (mRefTraj2[i].second.P - mRefTraj2[i - 1].second.P);
                            refP = p - refR * tmpP;

                            bAlignRef = true;
                            break;
                        }
                        continue;
                    }
                }
                mutex_ref_pose_.unlock();

            }
            mutex_cur_pose_.lock();

            curPQ.P = p;
            curPQ.Q = q;
            curT = tm;

            mutex_cur_pose_.unlock();
        }
    }

    void RosVisualization::trackedImageHandler(const sensor_msgs::Image::ConstPtr& img_msg)
    {
        cv_bridge::CvImageConstPtr ptr;
        ptr = cv_bridge::toCvCopy(img_msg);

        cv::Mat show_img = ptr->image;

        if (!show_img.empty()) {
            mutex_image_.lock();
            show_img.copyTo(image_);
            mutex_image_.unlock();
        }
    }

    void RosVisualization::refVioPose0Handler(const geometry_msgs::PointStamped::ConstPtr &odom) {

        if (odom) {
            mutex_ref_pose_.lock();

            double tmpTm;
            Eigen::Vector3d tmpP;
            Eigen::Matrix3d tmpR;
            Eigen::Quaterniond tmpQ(1,0,0,0);

            tmpTm = odom->header.stamp.toSec();

            tmpP(0) = odom->point.x;
            tmpP(1) = odom->point.y;
            tmpP(2) = odom->point.z;

            tmpR = tmpQ.toRotationMatrix();
            traj_point pq;
            pq.P = tmpP;
            pq.Q = tmpR;

            mRefTraj0.emplace_back(std::make_pair(tmpTm, pq));

            mutex_ref_pose_.unlock();

            if (mTraj.size() == 0 &&  modoTraj.size() == 0) {
                mutex_cur_pose_.lock();
                curPQ.P = tmpP;
                curPQ.Q = tmpQ;
                curT = tmpTm;
                mutex_cur_pose_.unlock();
            }
        }
    }

    void RosVisualization::refVioPose1Handler(const geometry_msgs::TransformStamped::ConstPtr &odom) {

        if (odom) {
            mutex_ref_pose_.lock();

            double tmpTm;
            Eigen::Vector3d tmpP;
            Eigen::Matrix3d tmpR;
            Eigen::Quaterniond tmpQ(1,0,0,0);

            tmpTm = odom->header.stamp.toSec();

            tmpP(0) = odom->transform.translation.x;
            tmpP(1) = odom->transform.translation.y;
            tmpP(2) = odom->transform.translation.z;

            tmpQ.w() = odom->transform.rotation.w;
            tmpQ.x() = odom->transform.rotation.x;
            tmpQ.y() = odom->transform.rotation.y;
            tmpQ.z() = odom->transform.rotation.z;

            tmpR = tmpQ.toRotationMatrix();
            traj_point pq;
            pq.P = tmpP;
            pq.Q = tmpR;

            mRefTraj1.emplace_back(std::make_pair(tmpTm, pq));

            mutex_ref_pose_.unlock();

            if (mTraj.size() == 0  &&  modoTraj.size() == 0) {
                mutex_cur_pose_.lock();
                curPQ.P = tmpP;
                curPQ.Q = tmpQ;
                curT = tmpTm;
                mutex_cur_pose_.unlock();
            }
        }
    }

    void RosVisualization::reflioPoseHandler(const geometry_msgs::PoseStamped::ConstPtr& odom)
    {
        if (odom) {
            mutex_ref_pose_.lock();

            double tmpTm;
            Eigen::Vector3d tmpP;
            Eigen::Matrix3d tmpR;
            Eigen::Quaterniond tmpQ(1,0,0,0);

            tmpTm = odom->header.stamp.toSec();

            tmpP(0) = odom->pose.position.x;
            tmpP(1) = odom->pose.position.y;
            tmpP(2) = odom->pose.position.z;

            tmpQ.w() = odom->pose.orientation.w;
            tmpQ.x() = odom->pose.orientation.x;
            tmpQ.y() = odom->pose.orientation.y;
            tmpQ.z() = odom->pose.orientation.z;

            tmpR = tmpQ.toRotationMatrix();
            traj_point pq;
            pq.P = tmpP;
            pq.Q = tmpR;

            mRefTraj2.emplace_back(std::make_pair(tmpTm, pq));

            mutex_ref_pose_.unlock();

            if (mTraj.size() == 0  &&  modoTraj.size() == 0) {
                mutex_cur_pose_.lock();
                curPQ.P = tmpP;
                curPQ.Q = tmpQ;
                curT = tmpTm;
                mutex_cur_pose_.unlock();
            }
        }
    }

    void RosVisualization::visualMapHandler(const sensor_msgs::PointCloud::ConstPtr &map_msg)
    {
        if(map_msg)
        {
            mutex_visual_map_.lock();

            m_visualMap.clear();

            for(auto& p:map_msg->points)
            {
                cv::Point3f pt;
                pt.x = p.x;
                pt.y = p.y;
                pt.z = p.z;
                m_visualMap.emplace_back(pt);
            }

            mutex_visual_map_.unlock();
        }
    }

    void RosVisualization::vioPoseHandler(const nav_msgs::Odometry::ConstPtr &odom) {

        //ROS_INFO("get 2 message success!");

        if (odom ) {
            geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;

            Eigen::Vector3d p(odom->pose.pose.position.x,
                              odom->pose.pose.position.y,
                              odom->pose.pose.position.z);
            Eigen::Quaterniond q;
            q.w() = geoQuat.w;
            q.x() = geoQuat.x;
            q.y() = geoQuat.y;
            q.z() = geoQuat.z;

            p = initQ*p;
            q = initQ*q;

            Eigen::Vector3d ypr = R2ypr(q.toRotationMatrix());

            double tm = odom->header.stamp.toSec();

            mutex_pose_.lock();
            traj_point pq;
            pq.P = p;
            pq.Q = q;

            modoTraj.emplace_back(std::make_pair(tm, pq));
            mutex_pose_.unlock();

//            ROS_INFO("PQ:%lf,%lf %lf %lf,%lf %lf %lf",tm,p(0),p(1),p(2),ypr(0),ypr(1),ypr(2));

            if(!bAlignRef)
            {
                // calculate
                mutex_ref_pose_.lock();

                if(mRefTraj0.size() > 1)
                {
                    for (int i = 1; i < mRefTraj0.size(); ++i) {
                        if (mRefTraj0[i].first > tm) {
                            double tmpPreTime = mRefTraj0[i - 1].first;
                            double tmpAftTime = mRefTraj0[i].first;

                            double scale = (tm - tmpPreTime) / (tmpAftTime - tmpPreTime);

                            Eigen::Quaterniond tmpQ = mRefTraj0[i - 1].second.Q.slerp(scale, mRefTraj0[i].second.Q);
                            tmpQ = q*(tmpQ.inverse());
                            refR = tmpQ.toRotationMatrix();

                            Eigen::Vector3d tmpP = mRefTraj0[i - 1].second.P + scale * (mRefTraj0[i].second.P - mRefTraj0[i - 1].second.P);
                            refP = p - refR * tmpP;

                            bAlignRef = true;
                            break;
                        }
                        continue;
                    }
                }

                if(mRefTraj1.size() > 1)
                {
                    for (int i = 1; i < mRefTraj1.size(); ++i) {
                        if (mRefTraj1[i].first > tm) {
                            double tmpPreTime = mRefTraj1[i - 1].first;
                            double tmpAftTime = mRefTraj1[i].first;

                            double scale = (tm - tmpPreTime) / (tmpAftTime - tmpPreTime);

                            Eigen::Quaterniond tmpQ = mRefTraj1[i - 1].second.Q.slerp(scale, mRefTraj1[i].second.Q);
                            tmpQ = q*(tmpQ.inverse());
                            refR = tmpQ.toRotationMatrix();

                            Eigen::Vector3d tmpP = mRefTraj1[i - 1].second.P + scale * (mRefTraj1[i].second.P - mRefTraj1[i - 1].second.P);
                            refP = p - refR * tmpP;

                            bAlignRef = true;
                            break;
                        }
                        continue;
                    }
                }

                mutex_ref_pose_.unlock();
            }

            mutex_cur_pose_.lock();

            curPQ.P = p;
            curPQ.Q = q;
            curT = tm;

            mutex_cur_pose_.unlock();
        }
    }

    void RosVisualization::spin() {
        ros::Rate rate(100);
        bool status = ros::ok();

        while (status) {
            ros::spinOnce();

            status = ros::ok();
            rate.sleep();
        }

    }
}