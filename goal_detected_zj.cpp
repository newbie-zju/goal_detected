#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Quaternion.h>
#include <goal_detected/Pose3D.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/AttitudeQuaternion.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <boost/concept_check.hpp>
#define Imagecenterx  376
#define Imagecentery  240
 //752*480
ros::Publisher goal_pose_pub;
ros::Publisher  T_vec_pub;
ros::Subscriber image_rect_sub;
ros::Subscriber quadrotorPos_sub;
ros::Subscriber quaternion_sub;	

using namespace std;
using namespace cv;
using namespace Eigen;
float yaw;

Mat src_img;
int threshold_param1=50;//35--20  55-P   35-V
int threshold_param2=45;
int blur_param=5;
int threshold_size=580;
dji_sdk::LocalPosition quadrotorPos;
ros::Time startTime;
ros::Time nowTime;
ros::Duration timer;
float Quater[4];
float Quater_last[4]={0};
//int count_quater = 0; //count error Q
VectorXd Body_to_Global( double arr[], float theta_angle, float Quater[] );
void drawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha, cv::Scalar color, int thickness = 1, int lineType = 8);
void blur_param_callback(int ,void*)
{ 
    int temp=blur_param%2;
    if(temp==0)  blur_param=blur_param-1;
}
void quadrotorPosCallback(const dji_sdk::LocalPosition::ConstPtr &msg)
{
	quadrotorPos.x = msg->x;
	quadrotorPos.y = msg->y;
	quadrotorPos.z = msg->z;
}
void quaternionCallback(const dji_sdk::AttitudeQuaternion::ConstPtr &msg)
{
	Quater[0] = msg->q0;
	Quater[1] = msg->q1;
	Quater[2] = msg->q2;
	Quater[3] = msg->q3;
	yaw = atan2(2.0 * (Quater[3] * Quater[0] + Quater[1] * Quater[2]) , - 1.0 + 2.0 * (Quater[0] * Quater[0] + Quater[1] * Quater[1]));
   	ROS_INFO("yaw = %f",yaw);
}
void image_rect_callback(const sensor_msgs::ImageConstPtr &msg)
{    
	double t=(double)cvGetTickCount(); 
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	cv_ptr->image.copyTo(src_img);

	vector<vector<Point> > contour;   //提取的轮廓
	vector<Vec4i> hierarchy; //提取的轮廓
	vector<vector<Point> > contour1;  //去除小面积轮廓
	vector<int> removal;    //去除边界轮廓编号
	vector<vector<Point2f> >   vertex;  ////最小面积矩形角点
	vector<vector<Point> > contour2;  //提取小块轮廓
	vector<Vec4i> hierarchy2;      //提取小块轮廓
	vector<Point2f > centerpt; //目标中心点
	vector<vector<Point2f>  > anglept; //存放用于计算方向的点组
	///////////////////////////////////////////////
	vector<vector<Point2f> > showpoints;
///////////////////////////////////////////////
	goal_detected::Pose3D pose;
	goal_detected::Pose3D T_vec_msg;
	float posex,posey,posez;
              float boxlength;//最小包围矩形的长
	float boxwidth;//最小包围矩形的宽
	double T_vec[3] ; //存放目标的三维坐标
	VectorXd GT_vec(3); //存放目标的大地坐标系下的x,y增量和小车运动方向与x轴的变换
	float goal_angle;//存放目标的方向
	int detected_flag;//检测到目标的标志位，0为无目标。
//---zj---
	Mat src_img32;
	src_img.convertTo(src_img32,CV_32FC3);//转换图像数据类型
	Mat gray_dst[3],threshold1,threshold2,threshold0,gaussian_dst;//通道分离
	split(src_img32,gray_dst);
	Mat contourimage=Mat::zeros(src_img.rows,src_img.cols,CV_8UC1);//建立空图
	threshold1 = -(-0.02661)*gray_dst[2]-(0.01342)*gray_dst[1]-(0.02487)*gray_dst[0]+(-0.9943)>-0.5;//r
	threshold2 = -(0.03109)*gray_dst[2]-(-0.05308)*gray_dst[1]-(0.03566)*gray_dst[0]+(0.3493)>-0.1;//g
	threshold0 = threshold1 | threshold2;
	morphologyEx(threshold0,threshold0,MORPH_OPEN,getStructuringElement(MORPH_ELLIPSE, Size(3,3)));//形态学滤波：场内去噪//#9.23
	morphologyEx(threshold0,gaussian_dst,MORPH_CLOSE,getStructuringElement(MORPH_ELLIPSE, Size(3,3)));//形态学滤波：场外去噪//#9.23
//---zj end---

/*
//反向二值化
	Mat threshold_dst=Mat::zeros(src_img.rows,src_img.cols,CV_8UC1);
	threshold(gray_dst[0],threshold_dst,threshold_param1,255,1);
//中值滤波
	Mat gaussian_dst;
	medianBlur(threshold_dst, gaussian_dst,blur_param); //大多数情况3也使用 
*/
//提取边框
	findContours(gaussian_dst,contour,hierarchy,RETR_CCOMP,CHAIN_APPROX_NONE);//RETR_LIST
	for(int i=0;i<contour.size();i++)
	{
	int size=contourArea(contour[i]);
	if (size>threshold_size)   //62000 620
	contour1.push_back(contour[i]);
	}
//找出边界疑似区域	
	if(contour1.size()>0)
	{
		for(int i=0;i<contour1.size();i++)
		{ 
	     		for(int j=0;j<contour1[i].size();j++)
			{
		  		Point2f temp;
		  		temp=contour1[i][j];
		  		if((temp.x<3)||(temp.y<3)||(temp.x > src_img.cols-3)||(temp.y > src_img.rows-3))
		   		{
					removal.push_back(i);
					break;
		   		}
		 	}
		}
	}
	else 
	{cout<<"no contours"<<endl;}
//去除边界疑似区域
	for(int i=removal.size()-1;i>=0;i--)
	{  		
       	contour1.erase(contour1.begin()+removal[i]);
	}
//drawContours(contourimage,contour1,-1,255,2,8);
	if(contour1.size()>0)
	{	
		for(int k=0;k<contour1.size();k++)
		{
			// 找出包围直线点的最小矩形
			RotatedRect box=minAreaRect(contour1[k]);
			Point2f verte[4];
			box.points(verte);

			boxlength=box.size.height;
			boxwidth=box.size.width;
			if(box.size.height<box.size.width)
			{ 
				boxlength=box.size.width;
				boxwidth=box.size.height;
			}
			float rate=boxlength/boxwidth;
			float distpoint=sqrt((verte[0].x-verte[1].x)*(verte[0].x-verte[1].x)
				+(verte[0].y-verte[1].y)*(verte[0].y-verte[1].y));
			if(distpoint>0.97*boxlength&&distpoint<1.03*boxlength)
			{
				Point temp;
				temp=verte[0];
				verte[0]=verte[1];
				verte[1]=verte[2];
				verte[2]=verte[3];
				verte[3]=temp;
			}
			vector<Point2f> vert;
			for(int i=0;i<4;i++)
			{ 
				vert.push_back(verte[i]);
			}
			//去除不符合矩形约束的轮廓	
			if(rate>1.1&&rate<2.4)
			{
				vertex.push_back(vert);
			}
		}
	}
//	else{cout<<"no contour1"<<endl;}
//对小矩形处理
	if(vertex.size()>0)
	{  
		//cout<<"goal number="<<vertex.size()<<endl; 
		for(int j=0;j<vertex.size();j++)
		{		
			Mat canonicalbox;
			Point2f pointRes[4];
			pointRes[0]=Point2f(0,boxwidth);
			pointRes[1]=Point2f(0,0);
			pointRes[2]=Point2f(boxlength,0);
			pointRes[3]=Point2f(boxlength,boxwidth);
			
			Point2f tempver[4];
			for(int i=0;i<4;i++)
			{tempver[i]=vertex[j][i];}
			
			Mat Matrix_Per=getPerspectiveTransform(tempver,pointRes);
			warpPerspective(src_img,canonicalbox,Matrix_Per,Size(boxlength,boxwidth),INTER_NEAREST);

			vector<Mat> canonicalbox_split ;
			split(canonicalbox,canonicalbox_split);
				  
			Mat  can_threshold_dst;
			//threshold(canonicalbox_split[0],can_threshold_dst,threshold_param1,255,1);
			can_threshold_dst = -(-0.0755)*canonicalbox_split[2]-(0.0376)*canonicalbox_split[1]-(0.063)*canonicalbox_split[0]+(-0.699) > -0.5;
			    
			Mat gaussian_dst2;
			medianBlur(can_threshold_dst, gaussian_dst2,3);

			findContours(gaussian_dst2,contour2,hierarchy2,RETR_CCOMP,CHAIN_APPROX_NONE);
			Mat canonicalbox2=Mat::zeros(can_threshold_dst.rows,can_threshold_dst.cols,CV_8UC1);
			//中心矩
			Moments moment=moments(can_threshold_dst);	
			Point2f moment_center=Point2f(moment.m10/moment.m00,moment.m01/moment.m00);
			//确定坐标系点顺序
			if(moment_center.y<canonicalbox.rows/2) //<canonicalbox.rows-moment_center.y
			{
				Point2f temp0,temp1;
				temp0=tempver[0];
				temp1=tempver[1];
				tempver[0]=tempver[2];
				tempver[1]=tempver[3];
				tempver[2]=temp0;
				tempver[3]=temp1;
			}
			//保存计算方向的点组
			Point2f p0((tempver[0].x+tempver[3].x)/2,(tempver[0].y+tempver[3].y)/2);
			Point2f p1((tempver[1].x+tempver[2].x)/2,(tempver[1].y+tempver[2].y)/2);
			vector<Point2f> angpt;
			angpt.push_back(p0);
			angpt.push_back(p1);
			anglept.push_back(angpt);
			//画出箭头
			//drawArrow(src_img,Point((tempver[0].x+tempver[3].x)/2,(tempver[0].y+tempver[3].y)/2),
			//Point((tempver[1].x+tempver[2].x)/2,(tempver[1].y+tempver[2].y)/2),20,15,Scalar(0,255,255),5);		
		}
	}
//求最近小车像素坐标与角度
	if(vertex.size()>0)
	{	
		for(int goal_ID=0;goal_ID<vertex.size();goal_ID++)
		{
		//求最近小车像素坐标
		// 	int min_centerx=0,min_centery=0,min_dst_center=1000,min_ID=0;
		// 	for(int i=0;i<vertex.size();i++)
		// 	{
		// 		float centerx=0,centery=0,dst_center=0;
		// 		for(int j=0;j<4;j++)
		// 		{
		// 			centerx+=vertex[i][j].x;
		// 			centery+=vertex[i][j].y;
		// 		}
		// 		vector<float>  centers;
		// 		centerx=centerx/4;			
		// 		centery=centery/4;
		// 		Point2f pt(centerx,centery);
		// 		centerpt.push_back(pt);
		// 		dst_center=sqrt((centerx-Imagecenterx)*(centerx-Imagecenterx)+(centery-Imagecentery)*(centery-Imagecentery));
		// 		if(dst_center<min_dst_center)
		// 		{
		// //			min_dst_center=dst_center;
		// 			min_ID=i;
		// 		}			
		// 	}
			float Markerlength=0.254,Markerwidth=0.1524;
			Mat Objpoints(4,3,CV_32FC1);
			Objpoints.at<float>(0,0)=-Markerlength/2;
			Objpoints.at<float>(0,1)=0;
			Objpoints.at<float>(0,2)=0;
			Objpoints.at<float>(1,0)=-Markerlength/2;
			Objpoints.at<float>(1,1)=Markerwidth;
			Objpoints.at<float>(1,2)=0;
			Objpoints.at<float>(2,0)=Markerlength/2;
			Objpoints.at<float>(2,1)=Markerwidth;
			Objpoints.at<float>(2,2)=0;
			Objpoints.at<float>(3,0)=Markerlength/2;
			Objpoints.at<float>(3,1)=0;
			Objpoints.at<float>(3,2)=0;

			Mat CameraMatrix(3,3,CV_32FC1);
			CameraMatrix.at<float>(0,0)=419.998;//fx
			CameraMatrix.at<float>(0,1)=0;
			CameraMatrix.at<float>(0,2)=361.793;//U0
			CameraMatrix.at<float>(1,0)=0;
			CameraMatrix.at<float>(1,1)=418.866;//fy
			CameraMatrix.at<float>(1,2)=247.059;//V0
			CameraMatrix.at<float>(2,0)=0;
			CameraMatrix.at<float>(2,1)=0;
			CameraMatrix.at<float>(2,2)=1;

			Mat Imgpoints(4,2,CV_32FC1);
			Imgpoints.at<float>(0,0)=vertex[goal_ID][0].x;
			Imgpoints.at<float>(0,1)=vertex[goal_ID][0].y;
			Imgpoints.at<float>(1,0)=vertex[goal_ID][1].x;
			Imgpoints.at<float>(1,1)=vertex[goal_ID][1].y;
			Imgpoints.at<float>(2,0)=vertex[goal_ID][2].x;
			Imgpoints.at<float>(2,1)=vertex[goal_ID][2].y;
			Imgpoints.at<float>(3,0)=vertex[goal_ID][3].x;
			Imgpoints.at<float>(3,1)=vertex[goal_ID][3].y;

			Mat distCoeffs(4,1,CV_32FC1);
			for(int i=0;i<4;i++)
				distCoeffs.at<float>(i,0)=0;
			Mat rvec,tvec;
			Mat Rvec,Tvec;
			solvePnP(Objpoints,Imgpoints,CameraMatrix,distCoeffs,rvec,tvec);
			tvec.convertTo(Tvec,CV_32F);
			rvec.convertTo(Rvec,CV_32F);

			for(int z=0;z<3;z++)
			{	
				T_vec[z]=Tvec.at<float>(z,0);
			}
			//cout<<"goal_center="<<centerpt[min_ID].x<<"  "<<centerpt[min_ID].y<<endl;
			//circle(src_img,Point(centerpt[min_ID].x,centerpt[min_ID].y),5,Scalar(250,0,0),-1);
			//circle(src_img,Point(376,240),5,Scalar(250,0,0),-1);
			float beta,k_angle;
			if((anglept[goal_ID][0].x-anglept[goal_ID][1].x)!=0)
			{
				k_angle=(anglept[goal_ID][0].y-anglept[goal_ID][1].y)/(anglept[goal_ID][0].x-anglept[goal_ID][1].x);
				beta=(atan(k_angle))*360/(2*3.1415926);
			}
			else
			{
				beta=0;
			}

			if(anglept[goal_ID][0].x<anglept[goal_ID][1].x)
			{
				goal_angle=beta+90;
			}
			else if((anglept[goal_ID][0].x>anglept[goal_ID][1].x))
			{
				goal_angle=beta-90;
			}
			else 
			{
				if(anglept[goal_ID][0].y<anglept[goal_ID][1].y)
				{
					goal_angle=180;
				}
				else 
				{
					goal_angle=0;
				}
			}     


			//  //涉及的从图像坐标系（x右y下）到机体坐标系的转换（x上y右）
			posex=-T_vec[1];
			posey=T_vec[0];
			posez=T_vec[2];
			//cout<<"[x,y]=   "<<posex<<"   "<<posey<<endl;


			//涉及的从图像坐标系（x右y下）到机体坐标系的转换（x上y右）
			double T_vec_tmp[3];
			T_vec_tmp[0] = -T_vec[1];
			T_vec_tmp[1] = T_vec[0];
			T_vec_tmp[2] = T_vec[2];

			if(fabs(Quater[0]*Quater[0]+Quater[1]*Quater[1]+Quater[2]*Quater[2]+Quater[3]*Quater[3] - 1)  > 0.2)
			{
				Quater[0] = Quater_last[0];
				Quater[1] = Quater_last[1];
				Quater[2] = Quater_last[2];
				Quater[3] = Quater_last[3];
				//count_quater = count_quater+1;
				//ROS_INFO("count= %d", count_quater);
			}
		//	ROS_INFO("Quater=%4.2f,%4.2f,%4.2f,%4.2f",Quater[0],Quater[1],Quater[2],Quater[3]);
			Quater_last[0] = Quater[0];
			Quater_last[1] = Quater[1];
			Quater_last[2] = Quater[2];
			Quater_last[3] = Quater[3];

			GT_vec = Body_to_Global( T_vec_tmp, goal_angle*3.1415926/180.0, Quater);
			pose.x.push_back(GT_vec(0)+ quadrotorPos.x);
			pose.y.push_back(GT_vec(1)+ quadrotorPos.y);
			pose.z.push_back(0);
			pose.theta.push_back( GT_vec(2));		
		}       
	}
	detected_flag=vertex.size();
	nowTime = ros::Time::now();
	timer = nowTime - startTime;
	if(detected_flag==0||quadrotorPos.z<1.3 || timer.toSec()<13)
	{
		T_vec[0]=0.0;
		T_vec[1]=0.0;
		T_vec[2]=0.0;
		goal_angle=0.0;
		detected_flag=0; 
	}
	//发送位置信息   
	pose.flag=detected_flag;
	

	goal_pose_pub.publish(pose);

/*
	drawContours(src_img,contour1,-1,255,2,8);
	namedWindow("show",CV_WINDOW_AUTOSIZE);
	imshow("show", src_img);
	createTrackbar("threshold_param1","show",&threshold_param1,100,NULL);
	createTrackbar("blur_param","show",&blur_param,10,blur_param_callback);
	createTrackbar("threshold_size","show",&threshold_size,2000,NULL);
	waitKey(1);
*/
	//image_rect_pub.publish( cv_ptr->toImageMsg());
	t=((double)cvGetTickCount() - t)/(cvGetTickFrequency()*1000);
	//cout<<"time="<<t<<endl;
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "goal_detected");
    ros::NodeHandle nh;
  //  image_rect_sub=nh.subscribe("/usb_cam/image_raw",1,&image_rect_callback);
    image_rect_sub=nh.subscribe("/mv_26804026/image_rect_color",1,&image_rect_callback); 
    goal_pose_pub=nh.advertise<goal_detected::Pose3D>("/goal_detected/goal_pose", 1);
    T_vec_pub = nh.advertise<geometry_msgs::Point>("T_vec", 1);
    quadrotorPos_sub = nh.subscribe("/dji_sdk/local_position", 10, quadrotorPosCallback);
    quaternion_sub = nh.subscribe("dji_sdk/attitude_quaternion", 10, quaternionCallback);
	startTime = ros::Time::now();
    ros::Rate loop_rate(20);
   while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
void drawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,cv::Scalar color, int thickness, int lineType)
 { 
	 const double PI = 3.1415926;
	 Point arrow;    
	 double angle = atan2((double)(pStart.y - pEnd.y), (double)(pStart.x - pEnd.x));  
	  line(img, pStart, pEnd, color, thickness, lineType);   
	  arrow.x = pEnd.x + len * cos(angle + PI * alpha / 180);     
	  arrow.y = pEnd.y + len * sin(angle + PI * alpha / 180);  
	  line(img, pEnd, arrow, color, thickness, lineType);   
	  arrow.x = pEnd.x + len * cos(angle - PI * alpha / 180);     
	  arrow.y = pEnd.y + len * sin(angle - PI * alpha / 180);    
	  line(img, pEnd, arrow, color, thickness, lineType);
}
VectorXd Body_to_Global( double Body_arry[], float theta_angle, float Quater[] )
{
	using namespace std;
	using namespace Eigen;
	
	double Gtheta_angle, theta_yaw;
	VectorXd Global_vector(3),  Result_vector(3), Body_vector(3);
	Body_vector(0) = Body_arry[0];
	Body_vector(1) = Body_arry[1];
	Body_vector(2) = Body_arry[2];
	MatrixXd Rotate(3, 3); 
	Rotate(0,0) = Quater[0] * Quater[0] + Quater[1] * Quater[1] - Quater[2] * Quater[2] - Quater[3] * Quater[3];
	Rotate(0,1) = 2 * ( Quater[1] * Quater[2] - Quater[0] * Quater[3]);
	Rotate(0,2) = 2 * ( Quater[1] * Quater[3] + Quater[0] * Quater[2]);
	Rotate(1,0) = 2 * ( Quater[1] * Quater[2] + Quater[0] * Quater[3]);
	Rotate(1,1) = Quater[0] * Quater[0] - Quater[1] * Quater[1] + Quater[2] * Quater[2] - Quater[3] * Quater[3];
	Rotate(1,2) = 2 * ( Quater[2] * Quater[3] - Quater[0] * Quater[1]);
	Rotate(2,0) = 2 * ( Quater[1] * Quater[3] - Quater[0] * Quater[2]);
	Rotate(2,1) = 2 * ( Quater[2] * Quater[3] + Quater[0] * Quater[1]);
	Rotate(2,2) = Quater[0] * Quater[0] - Quater[1] * Quater[1] - Quater[2] * Quater[2] + Quater[3] * Quater[3];
	
	//Rotate = Rotate.inverse();
	Global_vector = Rotate * Body_vector; //转换成NED坐标系下机体相对于小车的x y坐标的增量
	theta_yaw = atan2(2.0 * (Quater[3] * Quater[0] + Quater[1] * Quater[2]) , - 1.0 + 2.0 * (Quater[0] * Quater[0] + Quater[1] * Quater[1]));
	//ROS_INFO_THROTTLE(1,"theta_yaw: %f",theta_yaw);
	Result_vector(0) = Global_vector(0);
	Result_vector(1) = Global_vector(1);
	Gtheta_angle = theta_yaw + (float) theta_angle;
	if(Gtheta_angle<-M_PI) Gtheta_angle += 2*M_PI;
	if(Gtheta_angle>M_PI) Gtheta_angle -= 2*M_PI;
	Result_vector(2) = Gtheta_angle;
	return Result_vector;  //结果向量包含了机体相对于小车的x y坐标的增量和大地坐标系下的theta角
}
