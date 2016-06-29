#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace cv;
using namespace std;

/// Global variables
Mat src, src_gray;
char* pcd_file;

int maxCorners = 70;   

const char* source_window = "Image";

/// Function header_ì
vector<Point2f> central_mask( int, void* );
vector<Point2f> mask_right( int, void* );
vector<Point2f> mask_left( int, void* );

vector<double> comparison_pcl(vector<Point2f>);


// la funzione central_mask mi restituisce i corners trovati al cntro dell'immagine 2D
vector<Point2f> central_mask( int, void* )
{
  if( maxCorners < 1 ) { maxCorners = 1; }

  /// Parameters for Shi-Tomasi algorithm
  vector<Point2f> central_corners;
  double qualityLevel = 0.05;
  double minDistance = 12;    // distanza tra i vari corner (di default era a 10)
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  /// Copy the source image
  Mat copy, image_roi;
  copy = src.clone();
  Mat mask_cent = Mat::zeros(src.size(), CV_8UC1);
  Mat result;
 
  vector< vector<Point> >  co_ordinates;
  co_ordinates.push_back(vector<Point>());

  // maschera centrale ricorda che i punti sono definiti in senso antiorario

  Point P1((src.cols/2)-79,0),P2((src.cols/6)+1,src.rows),P3((src.cols-(src.cols/6))-1,src.rows),P4((src.cols/2)+79,0);   // i punti sono ordinati in senso antiorario
  
  co_ordinates[0].push_back(P1);
  co_ordinates[0].push_back(P2);
  co_ordinates[0].push_back(P3);
  co_ordinates[0].push_back(P4);

  drawContours( mask_cent,co_ordinates,0, Scalar(255),CV_FILLED, 8 );

  // imshow("maschera centrale", mask_cent);

  // conversione della maschera da 1 a 3 canali
  Mat mask_3, final_mask;
  cvtColor(mask_cent, mask_3, CV_GRAY2BGR,3);  // il primo argomento è la mia matrice di partenza, il secondo è la maschera di destinazione,
                                               // il terzo la conversione e poi il numero di canali che voglio
  bitwise_and(src, mask_3, result);
  
  cvtColor(result, final_mask, CV_BGR2GRAY, 1);

  cout<<"number of channel:" << final_mask.channels() << " tipo:" << final_mask.type() << final_mask.size() <<" depth:" << final_mask.depth() << endl;

  /// Apply corner detection
  goodFeaturesToTrack( src_gray,
               central_corners,
               maxCorners,
               qualityLevel,
               minDistance,
               final_mask,
               blockSize,
               useHarrisDetector,
               k );

  // int cornersSize = corners.size();
  // for(int k=0; k<cornersSize; k++)
  // { //goes through all cv::Point2f in the vector
  //   float x = corners[k].x;   //first value
  //   float y = corners[k].y;   //second value
  //               //stuff
  //   cout << "valore di coordinata x : " << x << endl;
  // }

  /// Draw corners detected
  cout<<"** Number of corners detected: "<<central_corners.size()<<endl;
  int r = 4;
  for( size_t i = 0; i < central_corners.size(); i++ )
     // { circle( copy, corners[i], r, Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)), -1, 8, 0 ); } 
     { circle( copy, central_corners[i], r, Scalar((0), (255), (255)), 1, 8, 0 ); }         // ricorda scalar mi permette di scegliere un 

  /// Show what you got
  namedWindow( source_window, WINDOW_AUTOSIZE );
  imshow( source_window, copy );

  return(central_corners);
}

vector<Point2f> mask_right( int, void* )
{
  if( maxCorners < 1 ) { maxCorners = 1; }

  /// Parameters for Shi-Tomasi algorithm
  vector<Point2f> corners_dx;
  double qualityLevel = 0.05;
  double minDistance = 12;    // distanza tra i vari corner (di default era a 10)
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  /// Copy the source image
  Mat copy, image_roi;
  copy = src.clone();
  // Mat mask_sx = Mat::zeros(src.size(), CV_8UC1);
  Mat mask_dx = Mat::zeros(src.size(), CV_8UC1);
  Mat result;

  vector< vector<Point> >  co_ordinates;
  co_ordinates.push_back(vector<Point>());

  // // creazione della maschera laterale di dx
  
  Point P5((src.cols/2)+80,0),P6(src.cols-(src.cols/6),src.rows),P7(src.cols,src.rows),P8(src.cols,0); 

  co_ordinates[0].push_back(P5);
  co_ordinates[0].push_back(P6);
  co_ordinates[0].push_back(P7);
  co_ordinates[0].push_back(P8);
  drawContours( mask_dx,co_ordinates,0, Scalar(255),CV_FILLED, 8 );
  // imshow("maschera dx", mask_dx);
 
  Mat mask_3, final_mask;
  cvtColor(mask_dx, mask_3, CV_GRAY2BGR,3);  // il primo argomento è la mia matrice di partenza, il secondo è la maschera di destinazione,
                                               // il terzo la conversione e poi il numero di canali che voglio
  bitwise_and(src, mask_3, result);
  
  cvtColor(result, final_mask, CV_BGR2GRAY, 1);

  /// Apply corner detection
  goodFeaturesToTrack( src_gray,
               corners_dx,
               maxCorners,
               qualityLevel,
               minDistance,
               final_mask,
               blockSize,
               useHarrisDetector,
               k );
  cout<<"** Number of corners detected: "<< corners_dx.size()<<endl;
  int r = 4;
  for( size_t i = 0; i < corners_dx.size(); i++ )
     { circle( copy, corners_dx[i], r, Scalar((0), (255), (255)), 1, 8, 0 ); }         // ricorda scalar mi permette di scegliere un colore

  /// Show what you got
  namedWindow( source_window, WINDOW_AUTOSIZE );
  imshow( source_window, copy );

  return(corners_dx);
}

vector<Point2f> mask_left( int, void* )
{
  if( maxCorners < 1 ) { maxCorners = 1; }

  /// Parameters for Shi-Tomasi algorithm
  vector<Point2f> corners_sx;
  double qualityLevel = 0.05;
  double minDistance = 12;    // distanza tra i vari corner (di default era a 10)
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  /// Copy the source image
  Mat copy, image_roi;
  copy = src.clone();
  Mat mask_sx = Mat::zeros(src.size(), CV_8UC1);
  Mat result;

  vector< vector<Point> >  co_ordinates;
  co_ordinates.push_back(vector<Point>());
  // punti della maschera laterale sx

  Point P1(0,0),P2(0,src.rows),P3((src.cols/6),src.rows),P4((src.cols/2)-80,0);   // i punti sono ordinati in senso antiorario
  
  co_ordinates[0].push_back(P1);
  co_ordinates[0].push_back(P2);
  co_ordinates[0].push_back(P3);
  co_ordinates[0].push_back(P4);
  drawContours( mask_sx,co_ordinates,0, Scalar(255),CV_FILLED, 8 );
 
  Mat mask_3, final_mask;
  cvtColor(mask_sx, mask_3, CV_GRAY2BGR,3);  // il primo argomento è la mia matrice di partenza, il secondo è la maschera di destinazione,
                                               // il terzo la conversione e poi il numero di canali che voglio
  bitwise_and(src, mask_3, result);
  
  cvtColor(result, final_mask, CV_BGR2GRAY, 1);

  /// Apply corner detection
  goodFeaturesToTrack( src_gray,
               corners_sx,
               maxCorners,
               qualityLevel,
               minDistance,
               final_mask,
               blockSize,
               useHarrisDetector,
               k );
  cout<<"** Number of corners detected: "<<corners_sx.size()<<endl;
  int r = 4;
  for( size_t i = 0; i < corners_sx.size(); i++ )
     { circle( copy, corners_sx[i], r, Scalar((0), (255), (255)), 1, 8, 0 ); }         // ricorda scalar mi permette di scegliere un colore

  /// Show what you got
  namedWindow( source_window, WINDOW_AUTOSIZE );
  imshow( source_window, copy );

  return(corners_sx);
}

// funzione che mi carica l'immagine pcd e confronta i pixel con quelli ricavati dall'immagine 2D
// vector<double> comparison_pcl(vector<Point2f> corner, char* pcd_file)
vector<double> comparison_pcl(vector<Point2f> corner)
{
  vector<double> z_coordinate;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *cloud);  //* load the file
  // {
  //   PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
  //   // return (1);
  // }
    for(int k = 0; k < corner.size(); ++k)
    {
      pcl::PointXYZ pt = cloud->at(corner[k].x, corner[k].y);
      z_coordinate.push_back(pt.z); 
    }        
  return(z_coordinate);
}

bool central_check()
{
  vector<Point2f> central_corner = central_mask( 0, 0);
  // confronto con la pcd e estrazione della z
  vector<double> central_z = comparison_pcl(central_corner);
  cout << "lunghezza dello z_vector = " << central_z.size() << endl;  

  for (vector<double>::const_iterator h = central_z.begin(); h != central_z.end(); ++h)
  cout << "la profondità z dei corners è: " << *h << ' '<< endl;
  cout<<'\n'<<std::flush; 
  for(int i = 0; i < central_z.size(); i++)
  {
    if(central_z[i] > 1.5 && central_z[i] < 2)
      return false;
  }

  return true;
}

bool right_check()
{
  // estrazione corner maschera di dx e corrispondente profondità
  vector<Point2f> corner_dx = mask_right( 0, 0);
  vector<double> z_dx = comparison_pcl(corner_dx);

  for(int i = 0; i < z_dx.size(); i++)
  {
    if(z_dx[i] > 1.5 && z_dx[i] < 2)
      return false;
  }

  return true;
}

bool left_check()
{
  // estrazione corner maschera di dx e corrispondente profondità
  vector<Point2f> corner_sx = mask_left( 0, 0);
  vector<double> z_sx = comparison_pcl(corner_sx);

  for(int i = 0; i < z_sx.size(); i++)
  {
    if(z_sx[i] > 1.5 && z_sx[i] < 2)
      return false;
  }

  return true;
}

int check()
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

int main( int, char** argv )
{
  /// Load source image and convert it to gray
  src = imread( argv[1], 1 );
  pcd_file = argv[2];
  cvtColor( src, src_gray, COLOR_BGR2GRAY );

  /// Create Window
  namedWindow( source_window, WINDOW_AUTOSIZE );
  imshow( source_window, src );

  switch (check())
  {
    case 1 : 
     cout << "go ahead" << endl;
     break;
    case 2 :
    cout << "turn right" << endl;
    break;
    case 3 :
    cout << "turn left" << endl;
    break;
    case 4 :
    cout << "STOP" << endl;
    break;
  }
  cout << "sono arrivata qui" << endl;

  // cout << "lunghezza dello z_vector = " << z_coordinate.size() << endl;  

  // for (vector<double>::const_iterator h = z_coordinate.begin(); h != z_coordinate.end(); ++h)
  // cout << "la profondità z dei corners è: " << *h << ' '<< endl;
  // cout<<'\n';  

  waitKey(0);
  return(0);
}