#include "Cvision.h"

// constructor

Cvision::Cvision()
 :it_(nh_)    // initialization list
{
	
	std::string topic = nh_.resolveName("/camera/depth_registered/points");
	sub_pcl_ = nh_.subscribe(topic,1, &Cvision::newCloud, this);
	image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1, &Cvision::convertCvmat, this);
	flag_cloud_ = false;
    flag_cvImage_ = false;
    // process's image parameters
    maxCorners = 70;
    /// Parameters for Shi-Tomasi algorithm
    qualityLevel = 0.05;
  	minDistance = 12;    // distanza tra i vari corner (di default era a 10)
  	blockSize = 3;
  	useHarrisDetector = false;
  	k = 0.04;
}

// deconstructor
Cvision::~Cvision()
{
   // non fa niente
}

// ************ FUNZIONI **************
// callback to subscribe from PointCloud2 to PointCloud
void Cvision::newCloud(const sensor_msgs::PointCloud2::ConstPtr& message)
{
	if(flag_cloud_)
		return;
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
	//constantly copy cloud from stream into class scene_stream_ to be accessible for service callback
	pcl::fromROSMsg (*message, *tmp);
	// std::string home = std::getenv( "HOME" );
	// pcl::PCDWriter writer;
	// writer.writeBinaryCompressed ((home + "/prova.pcd").c_str(), *tmp);
	std::cout << "image_tmp: " << std::endl;
	// la funzione mi restituisce la cloud my_cloud
	flag_cloud_ = true;
	copyPointCloud(*tmp, my_cloud_);
	
}

// callback to subscribe from ROS image to OpenCv image
void Cvision::convertCvmat(const sensor_msgs::ImageConstPtr& msg)
{
	if(flag_cvImage_)
		return;
	cv_bridge::CvImagePtr cv_ptr;
     try
      {
         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from %s to 'mono8'.", e.what());
        return;
      }
      	std::cout << "ciaone : " << std::endl;

      flag_cvImage_ = true;
      my_cvImage_ = cv_ptr->image.clone();
      // cv::cvtColor( my_cvImage_, src_gray, CV_BGR2GRAY );
}

std::vector<cv::Point2f> Cvision::central_mask( int, void* )
{
  
  if( maxCorners < 1 ) { maxCorners = 1; }
  std::vector<cv::Point2f> central_corners;

  /// Copy the source image
  // Mat copy, image_roi;
  // copy = src.clone();
  cv::Mat mask_cent = cv::Mat::zeros(my_cvImage_.size(), CV_8UC1);
  
 
  std::vector<std::vector<cv::Point> >  co_ordinates;
  co_ordinates.push_back(std::vector<cv::Point>());

  // maschera centrale ricorda che i punti sono definiti in senso antiorario
  cv::Point P1((my_cvImage_.cols/2)-79,0),P2((my_cvImage_.cols/6)+1,my_cvImage_.rows),P3((my_cvImage_.cols-(my_cvImage_.cols/6))-1,my_cvImage_.rows),P4((my_cvImage_.cols/2)+79,0);   // i punti sono ordinati in senso antiorario
  
  co_ordinates[0].push_back(P1);
  co_ordinates[0].push_back(P2);
  co_ordinates[0].push_back(P3);
  co_ordinates[0].push_back(P4);

  cv::drawContours( mask_cent,co_ordinates,0, cv::Scalar(255),CV_FILLED, 8 );

  // conversione della maschera da 1 a 3 canali
  //cv::cvtColor(mask_cent, mask_3, CV_GRAY2BGR,3);  // il primo argomento è la mia matrice di partenza, il secondo è la maschera di destinazione,
                                               // il terzo la conversione e poi il numero di canali che voglio
  cv::bitwise_and(my_cvImage_, mask_cent, final_mask);
  //cv::cvtColor(result, final_mask, CV_BGR2GRAY, 1);

  /// Apply corner detection
  cv::goodFeaturesToTrack( my_cvImage_,
               central_corners,
               maxCorners,
               qualityLevel,
               minDistance,
               final_mask,
               blockSize,
               useHarrisDetector,
               k );
  // /// Show what you got
  // namedWindow( source_window, WINDOW_AUTOSIZE );
  // imshow( source_window, copy );
  return(central_corners);
}

std::vector<cv::Point2f> Cvision::mask_right( int, void* )
{
  if( maxCorners < 1 ) { maxCorners = 1; }
  // corners return by function
  std::vector<cv::Point2f> corners_dx;

  /// Copy the source image
  // Mat copy, image_roi;
  // copy = src.clone();
  cv::Mat mask_dx = cv::Mat::zeros(my_cvImage_.size(), CV_8UC1);
  // cv::Mat result;

  std::vector<std::vector<cv::Point> >  co_ordinates;
  co_ordinates.push_back(std::vector<cv::Point>());

  // // creazione della maschera laterale di dx
  
  cv::Point P5((my_cvImage_.cols/2)+80,0),P6(my_cvImage_.cols-(my_cvImage_.cols/6),my_cvImage_.rows),P7(my_cvImage_.cols,my_cvImage_.rows),P8(my_cvImage_.cols,0); 

  co_ordinates[0].push_back(P5);
  co_ordinates[0].push_back(P6);
  co_ordinates[0].push_back(P7);
  co_ordinates[0].push_back(P8);
  cv::drawContours( mask_dx,co_ordinates,0, cv::Scalar(255),CV_FILLED, 8 );
  // cv::cvtColor(mask_dx, mask_3, CV_GRAY2BGR,3);  // il primo argomento è la mia matrice di partenza, il secondo è la maschera di destinazione,
                                               // il terzo la conversione e poi il numero di canali che voglio
  cv::bitwise_and(my_cvImage_, mask_dx, final_mask);
  // cv::cvtColor(result, final_mask, CV_BGR2GRAY, 1);

  /// Apply corner detection
  cv::goodFeaturesToTrack( my_cvImage_,
               corners_dx,
               maxCorners,
               qualityLevel,
               minDistance,
               final_mask,
               blockSize,
               useHarrisDetector,
               k );
  // cout<<"** Number of corners detected: "<< corners_dx.size()<<endl;
  // int r = 4;
  // for( size_t i = 0; i < corners_dx.size(); i++ )
  //    { circle( copy, corners_dx[i], r, Scalar((0), (255), (255)), 1, 8, 0 ); }         // ricorda scalar mi permette di scegliere un colore

  // /// Show what you got
  // namedWindow( source_window, WINDOW_AUTOSIZE );
  // imshow( source_window, copy );

  return(corners_dx);
}

std::vector<cv::Point2f> Cvision::mask_left( int, void* )
{
  if( maxCorners < 1 ) { maxCorners = 1; }
  std::vector<cv::Point2f> corners_sx;
  /// Copy the source image
  // Mat copy, image_roi;
  // copy = src.clone();
  cv::Mat mask_sx = cv::Mat::zeros(my_cvImage_.size(), CV_8UC1);

  std::vector<std::vector<cv::Point> >  co_ordinates;
  co_ordinates.push_back(std::vector<cv::Point>());
  // punti della maschera laterale sx
  cv::Point P1(0,0),P2(0,my_cvImage_.rows),P3((my_cvImage_.cols/6),my_cvImage_.rows),P4((my_cvImage_.cols/2)-80,0);   // i punti sono ordinati in senso antiorario
  
  co_ordinates[0].push_back(P1);
  co_ordinates[0].push_back(P2);
  co_ordinates[0].push_back(P3);
  co_ordinates[0].push_back(P4);
  cv::drawContours( mask_sx,co_ordinates,0, cv::Scalar(255),CV_FILLED, 8 );
  // cv::cvtColor(mask_sx, mask_3, CV_GRAY2BGR,3);  // il primo argomento è la mia matrice di partenza, il secondo è la maschera di destinazione,
                                               // il terzo la conversione e poi il numero di canali che voglio
  cv::bitwise_and(my_cvImage_, mask_sx, final_mask);
  // cv::cvtColor(result, final_mask, CV_BGR2GRAY, 1);

  /// Apply corner detection
  cv::goodFeaturesToTrack( my_cvImage_,
               corners_sx,
               maxCorners,
               qualityLevel,
               minDistance,
               final_mask,
               blockSize,
               useHarrisDetector,
               k );
  return(corners_sx);
}

// function to extract z_coordinates from image
std::vector<double> Cvision::comparison_pcl(std::vector<cv::Point2f> corner)
{
  	flag_cloud_ = true;
  	std::vector<double> z_coordinate;
  	// pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *my_cloud_);  //* load the cloud
    for(int k = 0; k < corner.size(); ++k)
    {
      	pcl::PointXYZ pt = my_cloud_.at(corner[k].x, corner[k].y);
      	z_coordinate.push_back(pt.z); 
    }        
  	return(z_coordinate); 
}

bool Cvision::central_check()
{
  std::vector<cv::Point2f> central_corner = central_mask( 0, 0);
  // confronto con la pcd e estrazione della z
  std::vector<double> central_z = comparison_pcl(central_corner);
  std::cout << "lunghezza dello z_vector = " << central_z.size() << std::endl;  

  for (std::vector<double>::const_iterator h = central_z.begin(); h != central_z.end(); ++h)
  std::cout << "la profondità z dei corners è: " << *h << ' '<< std::endl;
  std::cout<<'\n'<<std::flush; 
  for(int i = 0; i < central_z.size(); i++)
  {
    if(central_z[i] > 1.5 && central_z[i] < 2)
      return false;
  }

  return true;
}

bool Cvision::right_check()
{
  // estrazione corner maschera di dx e corrispondente profondità
  std::vector<cv::Point2f> corner_dx = mask_right( 0, 0);
  std::vector<double> z_dx = comparison_pcl(corner_dx);

  for(int i = 0; i < z_dx.size(); i++)
  {
    if(z_dx[i] > 1.5 && z_dx[i] < 2)
      return false;
  }

  return true;
}

bool Cvision::left_check()
{
  // estrazione corner maschera di dx e corrispondente profondità
  std::vector<cv::Point2f> corner_sx = mask_left( 0, 0);
  std::vector<double> z_sx = comparison_pcl(corner_sx);

  for(int i = 0; i < z_sx.size(); i++)
  {
    if(z_sx[i] > 1.5 && z_sx[i] < 2)
    
      return false;
  }

  return true;
}

int Cvision::processImage()
{
  if(central_check())
    return 1;
  else if (right_check())
    return 2;
  else if (left_check())
    return 3;
  else 
    return 4;

}