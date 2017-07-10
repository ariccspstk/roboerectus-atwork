#include "aricc_2d_vision/asad_vision.h"


namespace aricc_2d_vision{

  void ObjectDetector::onInit(){
    ConnectionBasedNodelet::onInit();

    pub_contours_ = 
      advertise<aricc_vision_msgs::ContourArray>(*pnh_,"output/contours", 1);
    pub_convex_contours_ = 
      advertise<aricc_vision_msgs::ContourArray>(*pnh_,"output/convex", 1);
    pub_poly_contours_ = 
      advertise<aricc_vision_msgs::ContourArray>(*pnh_,"output/poly", 1);
    pub_debug_image_ = 
      advertise<sensor_msgs::Image>(*pnh_,"output/test", 1);

  }

  void ObjectDetector::subscribe(){
    sub_ = pnh_->subscribe("input", 1, &ObjectDetector::execute, this);
  }

  void ObjectDetector::unsubscribe(){
    sub_.shutdown();
  }
  
  void ObjectDetector::pubResult(std_msgs::Header header){
    aricc_vision_msgs::ContourArray msg_contour_array;
    aricc_vision_msgs::ContourArray msg_convex_array;
    aricc_vision_msgs::ContourArray msg_poly_array;

    std::vector <std::vector<cv::Point> >::iterator it 
      = contours_.begin();
    for( ; it !=contours_.end(); ++it){
      aricc_vision_msgs::Contour msg;
      for(size_t i = 0; i < it->size(); ++i){
        geometry_msgs::Point p;
        p.x = it->at(i).x;
        p.y = it->at(i).y;
        msg.points.push_back(p);
      }
      msg_contour_array.contours.push_back(msg);
    }

    it = convex_contours_.begin();
    for( ; it !=convex_contours_.end(); ++it){
      aricc_vision_msgs::Contour msg;
      for(size_t i = 0; i < it->size(); ++i){
        geometry_msgs::Point p;
        p.x = it->at(i).x;
        p.y = it->at(i).y;
        msg.points.push_back(p);
      }
      msg_convex_array.contours.push_back(msg);
    }

    it = poly_contours_.begin();
    for( ; it !=poly_contours_.end(); ++it){
      aricc_vision_msgs::Contour msg;
      for(size_t i = 0; i < it->size(); ++i){
        geometry_msgs::Point p;
        p.x = it->at(i).x;
        p.y = it->at(i).y;
        msg.points.push_back(p);
      }
      msg_poly_array.contours.push_back(msg);
    }
    
    msg_contour_array.header = header;
    msg_convex_array.header = header;
    msg_poly_array.header = header;
    msg_contour_array.header.stamp = ros::Time::now();
    msg_convex_array.header.stamp = ros::Time::now();
    msg_poly_array.header.stamp = ros::Time::now();
    pub_contours_.publish(msg_contour_array);
    pub_convex_contours_.publish(msg_convex_array);
    pub_poly_contours_.publish(msg_poly_array);

    pub_debug_image_.publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, colorImage).toImageMsg());

  }
  
  void ObjectDetector::execute(
    const sensor_msgs::Image::ConstPtr& image_msg){
    boost::mutex::scoped_lock lock(mutex_);
    contours_.clear();
    convex_contours_.clear();
    poly_contours_.clear();
    hierarchy_.clear();

//    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
//      image_msg, image_msg->encoding);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
      image_msg, sensor_msgs::image_encodings::BGR8);
     cv::Mat input = cv_ptr->image;    

    img_width_ = input.cols;
    img_height_ = input.rows;

    colorImage = input.clone();

    getContours(THRESHOLD, false);
    getContours(THRESHOLD, true);

    pubResult(image_msg->header);  
  }

cv::Mat ObjectDetector::quantize(cv::Mat inputImg, bool gray)
{
    int windowSize = 5;

    erode(inputImg, inputImg, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(windowSize, windowSize)) );
    dilate(inputImg, inputImg, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(windowSize, windowSize)) );

    dilate(inputImg, inputImg, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(windowSize, windowSize)) );
    erode(inputImg, inputImg, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(windowSize, windowSize)) );


    //int coef = 107;//125;//107;//150;
    //int coef = !gray ? 150 : 50;
    if (gray)
    {
        int coef = 200;
        cv::Mat gray_quantized = inputImg/coef;
        gray_quantized = gray_quantized*coef;
        //imshow ("Gray Quantized", gray_quantized);
        return gray_quantized;
    }
    if (!gray)
    {
        int coef = 150;
        cv::Mat quantized = inputImg/coef;
        quantized = quantized*coef;
        //imshow("Quantizied", quantized);
        return quantized;
    }

    //cv::Mat crazy = quantized - inputImg;

    //return !gray ? quantized : crazy;
    //return quantized;
    //return crazy;
}

void ObjectDetector::saveImage(cv::Mat img, haar_sample_type type, int count)
{
    char* strType;
    char* filename;

    switch (type)
    {
    case POSITIVE:
        strType = "POS";
        break;
    case NEGATIVE:
        strType = "NEG";
        break;
    }

    for (int i=0; i<count; i++)
    {
        sprintf(filename, "%s[%d].jpg", strType, i);
        imwrite(filename, img);
    }

}

void ObjectDetector::saveImage(cv::Mat img, int index)
{
    char filename[30];

    sprintf(filename, "test/contours[%d].jpg", index);
    imwrite(filename, img);
}

void ObjectDetector::getContours(get_contour_type type, bool gray)
{
    cv::Mat grayImage;
    cv::Mat thresholdImage;
    cv::Mat drawing;
    cv::RNG rng(12345);
    int thresh = 100;//100;
    int threshGray = 150;
    int maxThresh = 255;
    double* objectArea;
    //double* objectAngle;

    char printInfoMotor_V[30] = "";
    char printInfoMotor_H[30] = "";
    char printInfoMotor_T[30] = "";
    char printInfoMotor_U[30] = "";

    char printInfoM20[30] = "";
    char printInfoM20_U[30] = "";

    char printInfoM30[30] = "";
    char printInfoM30_U[30] = "";

    char printInfoF20_20_B_V[30] = "";
    char printInfoF20_20_B_H[30] = "";
    char printInfoF20_20_B_T[30] = "";
    char printInfoF20_20_B_U[30] = "";

    char printInfoF20_20_G_V[30] = "";
    char printInfoF20_20_G_H[30] = "";
    char printInfoF20_20_G_T[30] = "";
    char printInfoF20_20_G_U[30] = "";

    char printInfoS40_40_B_V[30] = "";
    char printInfoS40_40_B_H[30] = "";
    char printInfoS40_40_B_T[30] = "";
    char printInfoS40_40_B_U[30] = "";

    char printInfoS40_40_G_V[30] = "";
    char printInfoS40_40_G_H[30] = "";
    char printInfoS40_40_G_T[30] = "";
    char printInfoS40_40_G_U[30] = "";

    char printInfoM20_100_V[30] = "";
    char printInfoM20_100_H[30] = "";
    char printInfoM20_100_T[30] = "";
    char printInfoM20_100_U[30] = "";

    char printInfoR20_V[30] = "";
    char printInfoR20_H[30] = "";
    char printInfoR20_T[30] = "";
    char printInfoR20_U[30] = "";

    char printInfoAxis_V[30] = "";
    char printInfoAxis_H[30] = "";
    char printInfoAxis_T[30] = "";
    char printInfoAxis_U[30] = "";

    char printInfoBearing_Box[30] = "";
    char printInfoBearing_Box_U[30] = "";

    char printInfoBearing[30] = "";
    char printInfoBearing_U[30] = "";

    char printInfoDistance_Tube[30] = "";
    char printInfoDistance_Tube_U[30] = "";

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    std::vector<std::vector<cv::Point> > contoursMotor;
    std::vector<std::vector<cv::Point> > contoursMotor_U;

    std::vector<std::vector<cv::Point> > contoursM20;
    std::vector<std::vector<cv::Point> > contoursM20_U;

    std::vector<std::vector<cv::Point> > contoursM30;
    std::vector<std::vector<cv::Point> > contoursM30_U;

    std::vector<std::vector<cv::Point> > contoursF20_20_B;
    std::vector<std::vector<cv::Point> > contoursF20_20_B_U;

    std::vector<std::vector<cv::Point> > contoursF20_20_G;
    std::vector<std::vector<cv::Point> > contoursF20_20_G_H;
    std::vector<std::vector<cv::Point> > contoursF20_20_G_T;
    std::vector<std::vector<cv::Point> > contoursF20_20_G_U;

    std::vector<std::vector<cv::Point> > contoursS40_40_B;
    std::vector<std::vector<cv::Point> > contoursS40_40_B_U;

    std::vector<std::vector<cv::Point> > contoursS40_40_G;
    std::vector<std::vector<cv::Point> > contoursS40_40_G_U;

    std::vector<std::vector<cv::Point> > contoursM20_100;
    std::vector<std::vector<cv::Point> > contoursM20_100_U;

    std::vector<std::vector<cv::Point> > contoursR20;
    std::vector<std::vector<cv::Point> > contoursR20_U;

    std::vector<std::vector<cv::Point> > contoursAxis;
    std::vector<std::vector<cv::Point> > contoursAxis_U;

    std::vector<std::vector<cv::Point> > contoursBearing_Box;
    std::vector<std::vector<cv::Point> > contoursBearing_Box_U;

    std::vector<std::vector<cv::Point> > contoursBearing;
    std::vector<std::vector<cv::Point> > contoursBearing_U;

    std::vector<std::vector<cv::Point> > contoursDistance_Tube;
    std::vector<std::vector<cv::Point> > contoursDistance_Tube_U;

    objData* objectInfo;

    double* scoreMotor;
    double* scoreMotor_U;
    double* scoreM20;
    double* scoreM20_U;
    double* scoreM30;
    double* scoreM30_U;
    double* scoreF20_20_B;
    double* scoreF20_20_B_U;
    double* scoreF20_20_G;
    double* scoreF20_20_G_H;
    double* scoreF20_20_G_T;
    double* scoreF20_20_G_U;
    double* scoreS40_40_B;
    double* scoreS40_40_B_U;
    double* scoreM20_100;
    double* scoreM20_100_U;
    double* scoreR20;
    double* scoreR20_U;
    double* scoreAxis;
    double* scoreAxis_U;
    double* scoreBearing_Box;
    double* scoreBearing_Box_U;
    double* scoreBearing;
    double* scoreBearing_U;
    double* scoreDistance_Tube;
    double* scoreDistance_Tube_U;
    double* scoreS40_40_G;
    double* scoreS40_40_G_U;


    if (gray)
    {
        cv::Mat input = adjustLight(colorImage);
        //input = quantize(input, gray);
        input = selectedColores(input, gray);
        cvtColor(input, grayImage, CV_BGR2GRAY);
    }

    if (!gray)
    {
        cv::Mat input = selectedColores(colorImage, gray);
        cvtColor(input, grayImage, CV_BGR2GRAY);
    }


    //cv::Mat input = quantize(colorImage, gray);
    //cvtColor(input, grayImage, CV_BGR2GRAY);

    //cv::cvtColor(colorImage, grayImage, CV_BGR2GRAY);

    blur(grayImage, grayImage, cv::Size(3, 3));

    //cv::imshow("Gray",grayImage);

    cv::Mat motor = cv::imread("/home/aricc/asad_img/Motor.png");
    cv::Mat motor_u = cv::imread("/home/aricc/asad_img/Motor_U.png");
    cv::Mat m20 = cv::imread("/home/aricc/asad_img/M20.png");
    cv::Mat m20_u = cv::imread("/home/aricc/asad_img/M20_U.png");
    cv::Mat m30 = cv::imread("/home/aricc/asad_img/M30.png");
    cv::Mat m30_u = cv::imread("/home/aricc/asad_img/M30_U.png");
    cv::Mat f20_20_b = cv::imread("/home/aricc/asad_img/F20_20_B.png");
    cv::Mat f20_20_b_u = cv::imread("/home/aricc/asad_img/F20_20_B_U.png");
    cv::Mat f20_20_g = cv::imread("/home/aricc/asad_img/F20_20_B.png");
    cv::Mat f20_20_g_h = cv::imread("/home/aricc/asad_img/F20_20_G_H.png");
    cv::Mat f20_20_g_t = cv::imread("/home/aricc/asad_img/F20_20_G_T.png");
    cv::Mat f20_20_g_u = cv::imread("/home/aricc/asad_img/F20_20_G_U.png");
    cv::Mat s40_40_b = cv::imread("/home/aricc/asad_img/S40_40_B.png");
    cv::Mat s40_40_b_u = cv::imread("/home/aricc/asad_img/S40_40_B_U.png");
    cv::Mat m20_100 = cv::imread("/home/aricc/asad_img/M20_100.png");
    cv::Mat m20_100_u = cv::imread("/home/aricc/asad_img/M20_100_U.png");
    cv::Mat r20 = cv::imread("/home/aricc/asad_img/R20.png");
    cv::Mat r20_u = cv::imread("/home/aricc/asad_img/R20_U.png");
    cv::Mat axis = cv::imread("/home/aricc/asad_img/Axis.png");
    cv::Mat axis_u = cv::imread("/home/aricc/asad_img/Axis_U.png");
    cv::Mat bearing_box = cv::imread("/home/aricc/asad_img/Bearing_Box.png");
    cv::Mat bearing_box_u = cv::imread("/home/aricc/asad_img/Bearing_Box_U.png");
    cv::Mat bearing = cv::imread("/home/aricc/asad_img/Bearing.png");
    cv::Mat bearing_u = cv::imread("/home/aricc/asad_img/Bearing_U.png");
    cv::Mat distance_tube = cv::imread("/home/aricc/asad_img/Distance_Tube.png");
    cv::Mat distance_tube_u = cv::imread("/home/aricc/asad_img/Distance_Tube_U.png");
    cv::Mat s40_40_g = cv::imread("/home/aricc/asad_img/S40_40_B.png");
    cv::Mat s40_40_g_u = cv::imread("/home/aricc/asad_img/S40_40_G_U.png");

    contoursAxis_U = getContours(axis_u);
    contoursM20_100 = getContours(m20_100);
    contoursM20_100_U = getContours(m20_100_u);
    contoursR20 = getContours(r20);
    contoursR20_U = getContours(r20_u);
    contoursMotor = getContours(motor);
    contoursMotor_U = getContours(motor_u);
    contoursF20_20_B = getContours(f20_20_b);
    contoursF20_20_G = getContours(f20_20_g);
    contoursM20_U = getContours(m20_u);
    contoursM30_U = getContours(m30_u);
    contoursBearing_Box = getContours(bearing_box);
    contoursBearing_Box_U = getContours(bearing_box_u);
    contoursBearing = getContours(bearing);
    contoursBearing_U = getContours(bearing_u);
    contoursDistance_Tube = getContours(distance_tube);
    contoursDistance_Tube_U = getContours(distance_tube_u);
    contoursF20_20_B_U = getContours(f20_20_b_u);
    contoursF20_20_G_U = getContours(f20_20_g_u);
    contoursAxis = getContours(axis);
    contoursS40_40_B_U = getContours(s40_40_b_u);
    contoursS40_40_B = getContours(s40_40_b);
    contoursS40_40_G = getContours(s40_40_g);
    contoursS40_40_G_U = getContours(s40_40_g_u);
    contoursM30 = getContours(m30);
    contoursF20_20_G_H = getContours(f20_20_g_h);
    contoursF20_20_G_T = getContours(f20_20_g_t);
    contoursM20 = getContours(m20);

    switch (type)
    {
    case THRESHOLD:
        threshold(grayImage, thresholdImage, thresh, maxThresh, cv::THRESH_BINARY);
        findContours(thresholdImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
        drawing = cv::Mat::zeros(thresholdImage.size(), CV_8UC3);
        break;
    case CANNY:
        cv::Mat cannyImage;
        Canny(grayImage, cannyImage, thresh, thresh*2, 3);
        findContours(cannyImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
        drawing = cv::Mat::zeros(cannyImage.size(), CV_8UC3);
        break;
    }


    std::vector<std::vector<cv::Point> > contours_poly(contours.size());
    std::vector<cv::Point2f>center( contours.size() );
    std::vector<float>radius( contours.size() );
    std::vector<cv::Rect> boundRect( contours.size() );

    scoreMotor = new double[contours.size()];
    scoreMotor_U = new double[contours.size()];
    scoreM20 = new double[contours.size()];
    scoreM20_U = new double[contours.size()];
    scoreM30 = new double[contours.size()];
    scoreM30_U = new double[contours.size()];
    scoreF20_20_B = new double[contours.size()];
    scoreF20_20_B_U = new double[contours.size()];
    scoreF20_20_G = new double[contours.size()];
    scoreF20_20_G_H = new double[contours.size()];
    scoreF20_20_G_T = new double[contours.size()];
    scoreF20_20_G_U = new double[contours.size()];
    scoreM20_100 = new double[contours.size()];
    scoreM20_100_U = new double[contours.size()];
    scoreR20 = new double[contours.size()];
    scoreR20_U = new double[contours.size()];
    scoreAxis = new double[contours.size()];
    scoreAxis_U = new double[contours.size()];
    scoreBearing_Box = new double[contours.size()];
    scoreBearing_Box_U = new double[contours.size()];
    scoreBearing = new double[contours.size()];
    scoreBearing_U = new double[contours.size()];
    scoreDistance_Tube = new double[contours.size()];
    scoreDistance_Tube_U = new double[contours.size()];
    scoreS40_40_B_U = new double[contours.size()];
    scoreS40_40_B = new double[contours.size()];
    scoreS40_40_G = new double[contours.size()];
    scoreS40_40_G_U = new double[contours.size()];

    objectInfo = new objData[contours.size()*3];

    objectArea = new double[contours.size()];
    //objectAngle = new double[contours.size()];

    cv::Mat object = colorImage.clone();

    cv::Mat hsvImage = colorImage.clone();
    cv::cvtColor(colorImage, hsvImage, CV_BGR2HSV);

    int objCount = -1;

    for (int i = 0; i < contours.size(); i++)
    {
        approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
        boundRect[i] = boundingRect( cv::Mat(contours_poly[i]) );
        minEnclosingCircle((cv::Mat)contours_poly[i], center[i], radius[i]);
        //objectArea = contourArea(contours_poly[i], false);
        objectArea[i] = boundRect[i].height * boundRect[i].width;
        //cv::RotatedRect rotated = cv::minAreaRect(cv::Mat(contours_poly[i]));
        //objectAngle[i]  = rotated.angle;

        float dis = getDistance(depthImage, center[i].x, center[i].y);
        cv::Vec3b col = getColor(colorImage, center[i].x, center[i].y);
        //cv::Vec3b col = getColor(hsvImage, center[i].x, center[i].y);
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

        cv::Scalar mean = cv::mean(colorImage(boundRect[i]));
        //cv::Scalar mean = cv::mean(colorImage(cv::circle()));
        //sprintf(printInfoMotor_V,"Motor= %i",col[0]);//col[3]);//mean[0]);
        //putText(object, printInfoMotor_V, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );

        if (!gray && center[i].y > 80)
        {
            scoreMotor[i] = cv::matchShapes(contoursMotor[1], contours[i], 1, 0.0);
            scoreMotor_U[i] = cv::matchShapes(contoursMotor_U[1], contours[i], 1, 0.0);
            scoreF20_20_B[i] = cv::matchShapes(contoursF20_20_B[1], contours[i], 1, 0.0);
            scoreF20_20_B_U[i] = cv::matchShapes(contoursF20_20_B_U[0], contours[i], 2, 0.0);
            scoreM20_100[i] = cv::matchShapes(contoursM20_100[1], contours[i], 1, 0.0);
            scoreM20_100_U[i] = cv::matchShapes(contoursM20_100_U[1], contours[i], 1, 0.0);
            scoreR20[i] = cv::matchShapes(contoursR20[1], contours[i], 1, 0.0);
            scoreR20_U[i] = cv::matchShapes(contoursR20_U[1], contours[i], 1, 0.0);
            scoreM30[i] = cv::matchShapes(contoursM30[0], contours[i], 1, 0.0);
            scoreM30_U[i] = cv::matchShapes(contoursM30_U[1], contours[i], 1, 0.0);
            scoreS40_40_B_U[i] = cv::matchShapes(contoursS40_40_B_U[1], contours[i], 1, 0.0);
            scoreS40_40_B[i] = cv::matchShapes(contoursS40_40_B[1], contours[i], 1, 0.0);

            sprintf(printInfoMotor_V,"Motor_V= %f",objectArea[i]);
            sprintf(printInfoMotor_H,"Motor_H= %f",objectArea[i]);
            sprintf(printInfoMotor_T,"Motor_T= %f",objectArea[i]);
//            sprintf(printInfoMotor_U,"Motor_U= %f",radius[i]);

            sprintf(printInfoF20_20_B_V,"F20_20_B_V= %f",scoreF20_20_B[i]);
            sprintf(printInfoF20_20_B_H,"F20_20_B_H= %f",scoreF20_20_B[i]);
            sprintf(printInfoF20_20_B_T,"F20_20_B_T= %f",radius[i]);
//            sprintf(printInfoF20_20_B_U,"F20_20_B_U= %f",objectArea[i]);

            sprintf(printInfoS40_40_B_V,"S40_40_B_V= %f",scoreS40_40_B[i]);
            sprintf(printInfoS40_40_B_H,"S40_40_B_H= %f",scoreS40_40_B[i]);
            sprintf(printInfoS40_40_B_T,"S40_40_B_T= %f",scoreS40_40_B[i]);
//            sprintf(printInfoS40_40_B_U,"S40_40_B_U= %f",scoreS40_40_B_U[i]);

            sprintf(printInfoM20_100_V,"M20_100_V= %f",scoreM20_100[i]);
            sprintf(printInfoM20_100_H,"M20_100_H= %f",radius[i]);
            sprintf(printInfoM20_100_T,"M20_100_T= %f",radius[i]);
//            sprintf(printInfoM20_100_U,"M20_100_U= %f",objectArea[i]);

            sprintf(printInfoR20_V,"R20_V= %f",scoreR20[i]);
            sprintf(printInfoR20_H,"R20_H= %f",objectArea[i]);
            sprintf(printInfoR20_T,"R20_T= %f",objectArea[i]);
//            sprintf(printInfoR20_U,"R20_U= %f",scoreR20_U[i]);

            sprintf(printInfoM30,"M30= %f",radius[i]);
//            sprintf(printInfoM30_U,"M30_U= %f",radius[i]);

            if ( (scoreMotor[i] < 0.09 && scoreMotor[i] != 0) && (radius[i] > 45 && radius[i] < 70) )
            {
                if ( (objectArea[i] < 8500 && objectArea[i] > 10) && (boundRect[i].height > boundRect[i].width) )
                {
                    putText(object, printInfoMotor_V, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .4, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = VERTICAL;
                    objectInfo[objCount].objDescription = MOTOR;
                }
                if ( (objectArea[i] < 8500 && objectArea[i] > 10) && (boundRect[i].height < boundRect[i].width) )
                {
                    putText(object, printInfoMotor_H, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = HORIZONTAL;
                    objectInfo[objCount].objDescription = MOTOR;
                }
                if (objectArea[i] > 8500)
                {
                    putText(object, printInfoMotor_T, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = TILTED;
                    objectInfo[objCount].objDescription = MOTOR;
                }
            }
            if ( (scoreMotor_U[i] < 0.08 && scoreMotor_U[i] != 0) && (radius[i] < 40 && radius[i] > 20) )
            {
                putText(object, printInfoMotor_U, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                rectangle(object, boundRect[i], color);
                objectInfo[++objCount].objX = center[i].x;
                objectInfo[objCount].objY = center[i].y;
                objectInfo[objCount].objOrientation = VERTICAL;
                objectInfo[objCount].objDescription = MOTOR;
            }

            if ( (scoreF20_20_B[i] < 1.2 && scoreF20_20_B[i] != 0) && (radius[i] > 55 && radius[i] < 80) )
            {
                if ( (objectArea[i] < 5000 && objectArea[i] > 10) && (boundRect[i].height > boundRect[i].width) )
                {
                    putText(object, printInfoF20_20_B_V, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = VERTICAL;
                    objectInfo[objCount].objDescription = F20_20_B;
                }
                if ( (objectArea[i] < 5000 && objectArea[i] > 10) && (boundRect[i].height < boundRect[i].width) )
                {
                    putText(object, printInfoF20_20_B_H, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = HORIZONTAL;
                    objectInfo[objCount].objDescription = F20_20_B;
                }
                if (objectArea[i] > 5000)
                {
                    putText(object, printInfoF20_20_B_T, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = TILTED;
                    objectInfo[objCount].objDescription = F20_20_B;
                }
            }
            if ( ((scoreF20_20_B_U[i] < 3.5 && scoreF20_20_B_U[i] > 2) && (radius[i] > 15 && radius[i] < 35)) || ((scoreF20_20_B_U[i] < 2 && scoreF20_20_B_U[i] > 1) && (radius[i] > 15 && radius[i] < 25)) )
            {
                putText(object, printInfoF20_20_B_U, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                rectangle(object, boundRect[i], color);
                objectInfo[++objCount].objX = center[i].x;
                objectInfo[objCount].objY = center[i].y;
                objectInfo[objCount].objOrientation = UP;
                objectInfo[objCount].objDescription = F20_20_B;
            }

            if ( (scoreS40_40_B[i] < 0.11 && scoreS40_40_B[i] != 0) && (radius[i] >= 60 && radius[i] < 85) )
            {
                if ( (objectArea[i] < 11000 && objectArea[i] > 10) && (boundRect[i].height > boundRect[i].width) )
                {
                    putText(object, printInfoS40_40_B_V, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = VERTICAL;
                    objectInfo[objCount].objDescription = S40_40_B;
                }
                if ( (objectArea[i] < 11000 && objectArea[i] > 10) && (boundRect[i].height < boundRect[i].width) )
                {
                    putText(object, printInfoS40_40_B_H, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = HORIZONTAL;
                    objectInfo[objCount].objDescription = S40_40_B;
                }
                if (objectArea[i] > 11000)
                {
                    putText(object, printInfoS40_40_B_T, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = TILTED;
                    objectInfo[objCount].objDescription = S40_40_B;
                }
            }
            if ( (scoreS40_40_B_U[i] < 0.2 && scoreS40_40_B_U[i] !=0) && (radius[i] > 45 && radius[i] < 55) )
            {
                putText(object, printInfoS40_40_B_U, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                rectangle(object, boundRect[i], color);
                objectInfo[++objCount].objX = center[i].x;
                objectInfo[objCount].objY = center[i].y;
                objectInfo[objCount].objOrientation = UP;
                objectInfo[objCount].objDescription = S40_40_B;
            }

            if ( (scoreM20_100[i] < 1.2 && scoreM20_100[i] != 0) && (radius[i] > 55 && radius[i] < 90) )
            {
                if ( (objectArea[i] < 10000 && objectArea[i] > 10) && (boundRect[i].height > boundRect[i].width) )
                {
                    putText(object, printInfoM20_100_V, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = VERTICAL;
                    objectInfo[objCount].objDescription = M20_100;
                }
                if ( (objectArea[i] < 10000 && objectArea[i] > 10) && (boundRect[i].height < boundRect[i].width) )
                {
                    putText(object, printInfoM20_100_H, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = HORIZONTAL;
                    objectInfo[objCount].objDescription = M20_100;
                }
                if (objectArea[i] > 10000)
                {
                    putText(object, printInfoM20_100_T, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = TILTED;
                    objectInfo[objCount].objDescription = M20_100;
                }
            }
            if ( (scoreM20_100_U[i] < 0.05 && scoreM20_100_U[i] != 0) && (radius[i] < 35 && radius[i] > 15) )
            {
                putText(object, printInfoM20_100_U, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                rectangle(object, boundRect[i], color);
                objectInfo[++objCount].objX = center[i].x;
                objectInfo[objCount].objY = center[i].y;
                objectInfo[objCount].objOrientation = UP;
                objectInfo[objCount].objDescription = M20_100;
            }

            if ( (scoreR20[i] < 0.09 && scoreR20[i] > 0) && (radius[i] < 45 && radius[i] > 25) )
            {
                if ( (objectArea[i] < 3100 && objectArea[i] > 10) && (boundRect[i].height > boundRect[i].width) )
                {
                    putText(object, printInfoR20_V, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = VERTICAL;
                    objectInfo[objCount].objDescription = R20;
                }
                if ( (objectArea[i] < 3100 && objectArea[i] > 10) && (boundRect[i].height < boundRect[i].width) )
                {
                    putText(object, printInfoR20_H, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = HORIZONTAL;
                    objectInfo[objCount].objDescription = R20;
                }
                if (objectArea[i] > 3100)
                {
                    putText(object, printInfoR20_T, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = TILTED;
                    objectInfo[objCount].objDescription = R20;
                }
            }
            if (scoreR20_U[i] < 0.9 && scoreR20_U[i] > 0.3)
            {
                putText(object, printInfoR20_U, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                rectangle(object, boundRect[i], color);
                objectInfo[++objCount].objX = center[i].x;
                objectInfo[objCount].objY = center[i].y;
                objectInfo[objCount].objOrientation = UP;
                objectInfo[objCount].objDescription = R20;
            }

            if ( (scoreM30[i] < 0.005 && scoreM30[i] != 0) && (radius[i] < 45 && radius[i] > 25) )
            {
                putText(object, printInfoM30, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                rectangle(object, boundRect[i], color);
                cv::rectangle(colorImage, boundRect[i], cv::Scalar(255, 255, 255), CV_FILLED);
                objectInfo[++objCount].objX = center[i].x;
                objectInfo[objCount].objY = center[i].y;
                objectInfo[objCount].objOrientation = VERTICAL;
                objectInfo[objCount].objDescription = M30;
            }
            if ( (scoreM30_U[i] < 0.1 && scoreM30_U[i] != 0) && (radius[i] > 35 && radius[i] < 45) )
            {
                putText(object, printInfoM30_U, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                rectangle(object, boundRect[i], color);
                objectInfo[++objCount].objX = center[i].x;
                objectInfo[objCount].objY = center[i].y;
                objectInfo[objCount].objOrientation = UP;
                objectInfo[objCount].objDescription = M30;
            }

            //putText(object, printInfoR20_V, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
        }
        if (gray && center[i].y > 80)
        {
            scoreF20_20_G[i] = cv::matchShapes(contoursF20_20_G[0], contours[i], 1, 0.0);
            scoreF20_20_G_H[i] = cv::matchShapes(contoursF20_20_G_H[0], contours[i], 1, 0.0);
            scoreF20_20_G_T[i] = cv::matchShapes(contoursF20_20_G_T[0], contours[i], 1, 0.0);
            scoreF20_20_G_U[i] = cv::matchShapes(contoursF20_20_G_U[1], contours[i], 1, 0.0);
            scoreAxis[i] = cv::matchShapes(contoursAxis[1], contours[i], 1, 0.0);
            scoreAxis_U[i] = cv::matchShapes(contoursAxis_U[1], contours[i], 1, 0.0);
            scoreM20[i] = cv::matchShapes(contoursM20[0], contours[i], 1, 0.0);
            scoreM20_U[i] = cv::matchShapes(contoursM20_U[1], contours[i], 1, 0.0);
            scoreBearing_Box[i] = cv::matchShapes(contoursBearing_Box[1], contours[i], 1, 0.0);
            scoreBearing_Box_U[i] = cv::matchShapes(contoursBearing_Box_U[1], contours[i], 1, 0.0);
            scoreBearing[i] = cv::matchShapes(contoursBearing[1], contours[i], 1, 0.0);
            scoreBearing_U[i] = cv::matchShapes(contoursBearing_U[1], contours[i], 1, 0.0);
            scoreDistance_Tube[i] = cv::matchShapes(contoursDistance_Tube[1], contours[i], 1, 0.0);
            scoreDistance_Tube_U[i] = cv::matchShapes(contoursDistance_Tube_U[1], contours[i], 1, 0.0);
            scoreS40_40_G[i] = cv::matchShapes(contoursS40_40_G[0], contours[i], 1, 0.0);
            scoreS40_40_G_U[i] = cv::matchShapes(contoursS40_40_G_U[0], contours[i], 1, 0.0);
//            scoreM30[i] = cv::matchShapes(contoursM30[0], contours[i], 1, 0.0);
//            scoreM30_U[i] = cv::matchShapes(contoursM30_U[1], contours[i], 1, 0.0);


            //sprintf(printInfoF20_20_G,"F20_20_G= %i|%i|%i",col[0],col[1],col[2]);
            sprintf(printInfoF20_20_G_V,"F20_20_G_V= %f",scoreF20_20_G[i]);
            sprintf(printInfoF20_20_G_H,"F20_20_G_H= %f",scoreF20_20_G[i]);
            sprintf(printInfoF20_20_G_T,"F20_20_G_T= %f",scoreF20_20_G[i]);
//            sprintf(printInfoF20_20_G_U,"F20_20_G_U= %f",radius[i]);

            sprintf(printInfoS40_40_G_V,"S40_40_G_V= %f",objectArea[i]);
            sprintf(printInfoS40_40_G_H,"S40_40_G_H= %f",scoreS40_40_G[i]);
            sprintf(printInfoS40_40_G_T,"S40_40_G_T= %f",scoreS40_40_G[i]);
//            sprintf(printInfoS40_40_G_U,"S40_40_G_U= %f",scoreS40_40_G_U[i]);

            sprintf(printInfoAxis_V,"Axis_V= %f",scoreAxis[i]);
            sprintf(printInfoAxis_H,"Axis_H= %f",objectArea[i]);
            sprintf(printInfoAxis_T,"Axis_T= %f",objectArea[i]);
//            sprintf(printInfoAxis_U,"Axis_U= %f",radius[i]);

            sprintf(printInfoM20,"M20= %f",radius[i]);
//            sprintf(printInfoM20_U,"M20_U= %f",objectArea[i]);

            sprintf(printInfoBearing_Box,"Bearing_Box= %f",radius[i]);
//            sprintf(printInfoBearing_Box_U,"Bearing_Box_U= %f",radius[i]);

            sprintf(printInfoBearing,"Bearing= %f",scoreBearing[i]);
//            sprintf(printInfoBearing_U,"Bearing_U= %f",scoreBearing_U[i]);

            sprintf(printInfoDistance_Tube,"Distance_Tube= %f",objectArea[i]);
//            sprintf(printInfoDistance_Tube_U,"Distance_Tube_U= %f",scoreDistance_Tube_U[i]);

//            sprintf(printInfoM30,"M30= %f",scoreM30[i]);
//            sprintf(printInfoM30_U,"M30_U= %f",radius[i]);


            if ( (scoreS40_40_G[i] < 5 && scoreS40_40_G[i] > 0) && (radius[i] > 70 && radius[i] < 100) )
            {
                if ( (objectArea[i] < 20000 && objectArea[i] > 10) && (boundRect[i].height > boundRect[i].width) )
                {
                    cv::Mat roi = grayImage(cv::Rect(boundRect[i].x, boundRect[i].y, boundRect[i].width, boundRect[i].height));
                    //char print[30] = "";
                    cv::Scalar mean = cv::mean(roi);
                    //sprintf(print,"Mean= %f",mean[0]);
                    //putText(object, print, cv::Point(center[i].x,center[i].y+50) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    if (mean[0] < 210 && mean[0] > 190)
                    {
                        putText(object, printInfoS40_40_G_V, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                        rectangle(object, boundRect[i], color);
                        objectInfo[++objCount].objX = center[i].x;
                        objectInfo[objCount].objY = center[i].y;
                        objectInfo[objCount].objOrientation = VERTICAL;
                        objectInfo[objCount].objDescription = S40_40_G;
                    }
                }
                if ( (objectArea[i] < 20000 && objectArea[i] > 10) && (boundRect[i].height < boundRect[i].width) )
                {
                    putText(object, printInfoS40_40_G_H, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = HORIZONTAL;
                    objectInfo[objCount].objDescription = S40_40_G;
                }
                if (objectArea[i] > 20000)
                {
                    putText(object, printInfoS40_40_G_T, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = TILTED;
                    objectInfo[objCount].objDescription = S40_40_G;
                }
            }

            if ( (scoreS40_40_G_U[i] < 0.9 && scoreS40_40_G_U[i] > 0) && (radius[i] > 10 && radius[i] < 30) )
            {
                putText(object, printInfoS40_40_G_U, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                rectangle(object, boundRect[i], color);
                objectInfo[++objCount].objX = center[i].x;
                objectInfo[objCount].objY = center[i].y;
                objectInfo[objCount].objOrientation = UP;
                objectInfo[objCount].objDescription = S40_40_G;
            }

            if ( (scoreF20_20_G[i] < 6 && scoreF20_20_G[i] > 2) && (radius[i] < 85 && radius[i] > 55) )
            {
                if ( (objectArea[i] < 7500 && objectArea[i] > 10) && (boundRect[i].height > boundRect[i].width) )
                {
                    putText(object, printInfoF20_20_G_V, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = VERTICAL;
                    objectInfo[objCount].objDescription = F20_20_G;
                }
                if ( (objectArea[i] < 7500 && objectArea[i] > 10) && (boundRect[i].height < boundRect[i].width) )
                {
                    putText(object, printInfoF20_20_G_H, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = HORIZONTAL;
                    objectInfo[objCount].objDescription = F20_20_G;
                }
                if (objectArea[i] > 7500)
                {
                    putText(object, printInfoF20_20_G_T, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = TILTED;
                    objectInfo[objCount].objDescription = F20_20_G;
                }
            }

            if ( (scoreF20_20_G_U[i] < 0.03 && scoreF20_20_G_U[i] > 0.02) && (radius[i] < 24))
            {
                putText(object, printInfoF20_20_G_U, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                rectangle(object, boundRect[i], color);
                objectInfo[++objCount].objX = center[i].x;
                objectInfo[objCount].objY = center[i].y;
                objectInfo[objCount].objOrientation = UP;
                objectInfo[objCount].objDescription = F20_20_G;
            }

            if ( (scoreAxis[i] < 0.5 && scoreAxis[i] != 0) && (radius[i] > 50 && radius[i] < 75) )
            {
                if ( (objectArea[i] < 6000 && objectArea[i] > 10) && (boundRect[i].height > boundRect[i].width) )
                {
                    putText(object, printInfoAxis_V, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = VERTICAL;
                    objectInfo[objCount].objDescription = AXIS;
                }
                if ( (objectArea[i] < 6000 && objectArea[i] > 10) && (boundRect[i].height < boundRect[i].width) )
                {
                    putText(object, printInfoAxis_H, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = HORIZONTAL;
                    objectInfo[objCount].objDescription = AXIS;
                }
                if (objectArea[i] > 6000)
                {
                    putText(object, printInfoAxis_T, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = TILTED;
                    objectInfo[objCount].objDescription = AXIS;
                }
            }
            if ( (scoreAxis_U[i] < 1.5 && scoreAxis_U[i] != 0) && (radius[i] > 15 && radius[i] < 30) )
            {
                putText(object, printInfoAxis_U, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                rectangle(object, boundRect[i], color);
                objectInfo[++objCount].objX = center[i].x;
                objectInfo[objCount].objY = center[i].y;
                objectInfo[objCount].objOrientation = UP;
                objectInfo[objCount].objDescription = AXIS;
            }

            if ( (scoreM20[i] < 0.2 && scoreM20[i] > 0) && (radius[i] > 15 && radius[i] < 30) )
            {
                cv::Mat roi = grayImage(cv::Rect(boundRect[i].x, boundRect[i].y, boundRect[i].width, boundRect[i].height));
                //char print[30] = "";
                cv::Scalar mean = cv::mean(roi);
                //sprintf(print,"Mean= %f",mean[0]);
                //putText(object, print, cv::Point(center[i].x,center[i].y+50) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                if (mean[0] > 100 && mean[0] < 130)
                {
                    putText(object, printInfoM20, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = VERTICAL;
                    objectInfo[objCount].objDescription = M20;
                }
            }
            if ( (scoreM20_U[i] < 0.3 && scoreM20_U[i] > 0) && (radius[i] > 1 && radius[i] < 10) )
            {
                putText(object, printInfoM20_U, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                rectangle(object, boundRect[i], color);
                objectInfo[++objCount].objX = center[i].x;
                objectInfo[objCount].objY = center[i].y;
                objectInfo[objCount].objOrientation = UP;
                objectInfo[objCount].objDescription = M20;
            }

            if ( (scoreBearing_Box[i] < 0.05 && scoreBearing_Box[i] > 0) && (radius[i] > 35 && radius[i] < 55) )
            {
                cv::Mat roi = grayImage(cv::Rect(boundRect[i].x, boundRect[i].y, boundRect[i].width, boundRect[i].height));
                //char print[30] = "";
                cv::Scalar mean = cv::mean(roi);
                //sprintf(print,"Mean= %f",mean[0]);
                //putText(object, print, cv::Point(center[i].x,center[i].y+50) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                if (mean[0] < 90)
                {
                    putText(object, printInfoBearing_Box, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = VERTICAL;
                    objectInfo[objCount].objDescription = BEARING_BOX;
                }
            }
            if ( (scoreBearing_Box_U[i] < 0.3 && scoreBearing_Box_U[i] > 0) && (radius[i] < 50 && radius[i] > 30) )
            {
                putText(object, printInfoBearing_Box_U, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                rectangle(object, boundRect[i], color);
                objectInfo[++objCount].objX = center[i].x;
                objectInfo[objCount].objY = center[i].y;
                objectInfo[objCount].objOrientation = UP;
                objectInfo[objCount].objDescription = BEARING_BOX;
            }

            if ( (scoreBearing[i] < 0.009 && scoreBearing[i] > 0) && (radius[i] > 15 && radius[i] < 35) )
            {
                cv::Mat roi = grayImage(cv::Rect(boundRect[i].x, boundRect[i].y, boundRect[i].width, boundRect[i].height));
                //char print[30] = "";
                cv::Scalar mean = cv::mean(roi);
                //sprintf(print,"Mean= %f",mean[0]);
                //putText(object, print, cv::Point(center[i].x,center[i].y+50) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                if (mean[0] < 100)
                {
                    putText(object, printInfoBearing, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = VERTICAL;
                    objectInfo[objCount].objDescription = BEARING;
                }
            }
            if ( (scoreBearing_U[i] < 1.3/*0.09*/ && scoreBearing_U[i] !=0) && (radius[i] > 15 && radius[i] < 30) )
            {
                putText(object, printInfoBearing_U, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                rectangle(object, boundRect[i], color);
                objectInfo[++objCount].objX = center[i].x;
                objectInfo[objCount].objY = center[i].y;
                objectInfo[objCount].objOrientation = UP;
                objectInfo[objCount].objDescription = BEARING;
            }

            if ( (scoreDistance_Tube[i] < 0.09 && scoreDistance_Tube[i] > 0) && (radius[i] < 35 && radius[i] > 5) /*&& (objectArea[i] > 500)*/ )
            {
                cv::Mat roi = grayImage(cv::Rect(boundRect[i].x, boundRect[i].y, boundRect[i].width, boundRect[i].height));
                char print[30] = "";
                cv::Scalar mean = cv::mean(roi);
                sprintf(print,"Mean= %f",mean[0]);
                putText(object, print, cv::Point(center[i].x,center[i].y+50) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                if (mean[0] > 200)
                {
                    putText(object, printInfoDistance_Tube, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                    rectangle(object, boundRect[i], color);
                    objectInfo[++objCount].objX = center[i].x;
                    objectInfo[objCount].objY = center[i].y;
                    objectInfo[objCount].objOrientation = VERTICAL;
                    objectInfo[objCount].objDescription = DISTANCE_TUBE;
                }
            }
            if ( (scoreDistance_Tube_U[i] < 2 && scoreDistance_Tube_U[i] > 0) && (radius[i] > 3 && radius[i] < 15) )
            {
                putText(object, printInfoDistance_Tube_U, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                rectangle(object, boundRect[i], color);
                objectInfo[++objCount].objX = center[i].x;
                objectInfo[objCount].objY = center[i].y;
                objectInfo[objCount].objOrientation = UP;
                objectInfo[objCount].objDescription = DISTANCE_TUBE;
            }

            /*if (scoreM30[i] < 0.0005 && scoreM30[i] != 0)
            {
                putText(object, printInfoM30, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                rectangle(object, boundRect[i], color);
            }
            if ( (scoreM30_U[i] < 0.1 && scoreM30_U[i] != 0) && (radius[i] > 35 && radius[i] < 45) )
            {
                putText(object, printInfoM30_U, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
                rectangle(object, boundRect[i], color);
            }*/


            //putText(object, printInfoM20, cv::Point(center[i].x,center[i].y) , cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2,8,false );
        }


        //sprintf(printInfo,"Area= %f",objectArea);

        color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        //color = cv::Scalar(255, 255, 255);
        cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
        //rectangle(object, boundRect[i], color);
        //saveImage(drawing, i);
        //drawing = cv::Mat::zeros(thresholdImage.size(), CV_8UC3);
        //if (objectArea < 10000 && objectArea > 100)
    }

    if (gray)
    {
        imshow("Contours Gray", drawing);
        imshow("Objects Gray", object);
        cv::moveWindow("Contours Gray", 720, 300);
        cv::moveWindow("Objects Gray", 720, 0);
    }
    else
    {
        imshow("Contours", drawing);
        imshow("Objects", object);
        cv::moveWindow("Objects", 0, 0);
        cv::moveWindow("Contours", 0, 300);
    }
}

std::vector<std::vector<cv::Point> > ObjectDetector::getContours(cv::Mat inputImage)
{
    cv::Mat grayImage;
    cv::Mat thresholdImage;
    cv::Mat drawing;

    cv::RNG rng(12345);

    int thresh = 100;
    int maxThresh = 255;

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cvtColor(inputImage, grayImage, CV_BGR2GRAY);
    blur(grayImage, grayImage, cv::Size(3, 3));
    //blur(inputImage, grayImage, cv::Size(3, 3));


    threshold(grayImage, thresholdImage, thresh, maxThresh, cv::THRESH_BINARY);
    findContours(thresholdImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
    drawing = cv::Mat::zeros(thresholdImage.size(), CV_8UC3);

    for (int i=0; i<contours.size(); i++)
    {
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        cv::drawContours(drawing, contours, 0, color, 2, 8, hierarchy, 0, cv::Point());
    }

    //cout << "SIZE: " << contours.cv::Size() << endl;
    //imshow("test", drawing);

    return contours;
}

bool ObjectDetector::compareObjects(cv::Mat source)
{
    cv::Mat sample = cv::imread("sample.png");

    cv::RNG rng(12345);

    std::vector<std::vector<cv::Point> > contoursSample;
    std::vector<std::vector<cv::Point> > contoursSource;
    double* score;

    cv::Mat sampleDraw;
    cv::Mat sourceDraw;


    contoursSample = getContours(sample);
    contoursSource = getContours(source);

    std::vector<std::vector<cv::Point> > contours_poly(contoursSource.size());
    std::vector<cv::Point2f>center( contoursSource.size() );
    std::vector<float>radius( contoursSource.size() );

    score = new double[contoursSource.size()];


    for (int i=0; i<contoursSource.size(); i++)
    {
        score[i] = cv::matchShapes(contoursSample[0], contoursSource[i], 1, 0.0);
    }
}

cv::Mat ObjectDetector::selectedColores(cv::Mat input, bool gray)
{
    cv::Mat HSV;
    cv::Mat threshGrey;
    cv::Mat threshBlack;
    cv::Mat dst;

    int windowSize = 5;

    //HSV = input.clone();

    HSV = quantize(input, gray);

    cv::cvtColor(HSV, HSV, CV_BGR2HSV);

    cv::inRange(HSV, cv::Scalar(0, 0, 150), cv::Scalar(0, 0, 200), threshGrey);//77,77,72<>163,216,147
    cv::inRange(HSV, cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0), threshBlack);//77,77,72<>163,216,147

    //cv::inRange(HSV, cv::Scalar(46, 8, 105), cv::Scalar(66, 26, 219), threshGrey);//77,77,72<>163,216,147
    //cv::inRange(HSV, cv::Scalar(0, 0, 26), cv::Scalar(132, 42, 58), threshBlack);//77,77,72<>163,216,147

    std::vector<cv::Mat> channelsGrey;
    channelsGrey.push_back(threshGrey);
    channelsGrey.push_back(threshGrey);
    channelsGrey.push_back(threshGrey);
    cv::Mat colorThreshGrey;
    cv::merge(channelsGrey,colorThreshGrey);

    std::vector<cv::Mat> channelsBlack;
    channelsBlack.push_back(threshBlack);
    channelsBlack.push_back(threshBlack);
    channelsBlack.push_back(threshBlack);
    cv::Mat colorThreshBlack;
    cv::merge(channelsBlack,colorThreshBlack);

    if (gray)
    {
        dst = colorThreshGrey;
        //imshow ("Grey Objects", dst);
        //cv::moveWindow("Grey Objects", 720, 0);
    }
    if (!gray)
    {
        dst = colorThreshBlack;
        //imshow("Black Objects", dst);
    }

    /*erode(dst, dst, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(windowSize, windowSize)) );
    dilate(dst, dst, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(windowSize, windowSize)) );

    dilate(dst, dst, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(windowSize, windowSize)) );
    erode(dst, dst, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(windowSize, windowSize)) );*/

    //imshow("Grey Objects", dst);

    return dst;
}

void ObjectDetector::removeBG(cv::Mat colorImg, cv::Mat depthImg, remove_background method)
{
    switch(method)
    {
    case BY_COLOR:
        break;
    case BY_DISTANCE:
        cv::Vec3b color;
        for (int i = 0; i < colorImg.rows; i++)
            for (int j = 0; j < colorImg.cols; j++)
            {
                dist = depthImg.at<unsigned short>(i, j)/10.0;
                //cout << "Dis: " << dist << "at " << "(" << i << ", " << j << ")" << endl;
                if ((dist > 30) && (dist < 100))//34
                {
                    //Vec3b color = image.at<cv::Vec3b>(cv::Point(i, j));
                    color[0] = 0;
                    color[1] = 0;
                    color[2] = 255;
                    colorImg.at<cv::Vec3b>(cv::Point(j, i)) = color;
                }
            }

        cv::imshow("Removed BG", colorImg);
        break;
    }
}

float ObjectDetector::getDistance(cv::Mat depthImage, int x, int y)
{
    dist = depthImage.at<unsigned short>(y, x)/10.0;

    return dist;
}

cv::Vec3b ObjectDetector::getColor(cv::Mat input, int x, int y)
{
    objColor = input.at<cv::Vec3b>(cv::Point(y, x));

    return objColor;
}

cv::Mat ObjectDetector::adjustLight(cv::Mat input)
{
    cv::Mat lab_image;
    cv::cvtColor(input, lab_image, CV_BGR2Lab);

    // Extract the L channel
    std::vector<cv::Mat> lab_planes(3);
    cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

    // apply the CLAHE algorithm to the L channel
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    cv::Mat dst;
    clahe->apply(lab_planes[0], dst);

    // Merge the the color planes back into an Lab image
    dst.copyTo(lab_planes[0]);
    cv::merge(lab_planes, lab_image);

   // convert back to RGB
   cv::Mat image_clahe;
   cv::cvtColor(lab_image, image_clahe, CV_Lab2BGR);

   // display the results  (you might also want to see lab_planes[0] before and after).
   //cv::imshow("image original", input);
   //cv::imshow("image CLAHE", image_clahe);

   return image_clahe;
}

}
  
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_2d_vision::ObjectDetector, nodelet::Nodelet);
