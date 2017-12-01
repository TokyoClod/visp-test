/*! \example tutorial-ibvs-4pts-display.cpp */
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpProjectionDisplay.h>
#include <visp3/vs/vpServoDisplay.h>

void display_trajectory(const vpImage<unsigned char> &I, std::vector<vpPoint> &point,
                        const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam);

void display_trajectory(const vpImage<unsigned char> &I, std::vector<vpPoint> &point,
                        const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam)
{
  static std::vector<vpImagePoint> traj[4];
  vpImagePoint cog;
  for (unsigned int i=0; i<4; i++) {
    // Project the point at the given camera position
    point[i].project(cMo);
    vpMeterPixelConversion::convertPoint(cam, point[i].get_x(), point[i].get_y(), cog);
    traj[i].push_back(cog);
  }
  for (unsigned int i=0; i<4; i++) {
    for (unsigned int j=1; j<traj[i].size(); j++) {
      vpDisplay::displayLine(I, traj[i][j-1], traj[i][j], vpColor::green);
    }
  }
}

int main()
{
  try {
    vpHomogeneousMatrix cdMo(0, 0, 1.06, 0, 0, 0);
    vpHomogeneousMatrix cMo(0, 0, 2.5,
                            0, 0, 0);

    std::vector<vpPoint> point;
    point.push_back( vpPoint(-0.5,-0.4, 0.34) );
    point.push_back( vpPoint( 0.3,-0.4, 0.34) );
    point.push_back( vpPoint( 0.3, 0.4, 0.34) );
    point.push_back( vpPoint(-0.5, 0.4, 0.34) );

    vpServo task ;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::MEAN);
    task.setLambda(0.5);

    vpFeaturePoint p[4], pd[4] ;
    for (unsigned int i = 0 ; i < 4 ; i++) {
      point[i].track(cdMo);
      vpFeatureBuilder::create(pd[i], point[i]);     
      //std::cout << "pd["<<i<<"]init" << pd[i][0] << ", " << pd[i][1] << ", " << pd[i][2] << std::endl;
      point[i].track(cMo);
      vpFeatureBuilder::create(p[i], point[i]);
      //std::cout << "p["<<i<<"]init" << p[i][0] << ", " << p[i][1] << ", " << p[i][2] << std::endl;
      task.addFeature(p[i], pd[i]);
    }

    for (unsigned int i = 0 ; i < 4; i++) {
        std::cout << "pd["<<i<<"]_init: " << pd[i][0] << ", " << pd[i][1] << ", " << pd[i][2] << std::endl;
    }
    for (unsigned int i = 0 ; i < 4; i++) {
        std::cout << "p["<<i<<"]_init: " << p[i][0] << ", " << p[i][1] << ", " << p[i][2] << std::endl;
    }
    std::cout<<std::endl;

    vpHomogeneousMatrix wMc, wMo;
    vpSimulatorCamera robot;
    robot.setSamplingTime(0.040);
    robot.getPosition(wMc);
    wMo = wMc * cMo;

    vpImage<unsigned char> Iint(480, 640, 255) ;
    vpImage<unsigned char> Iext(480, 640, 255) ;

    vpDisplayX displayInt(Iint, 0, 0, "Internal view");
    vpDisplayX displayExt(Iext, 670, 0, "External view");


    vpProjectionDisplay externalview;
    for (unsigned int i = 0 ; i < 4 ; i++)
      externalview.insert(point[i]) ;

    vpCameraParameters cam(268.52, 268.52, Iint.getWidth()/2, Iint.getHeight()/2);
    vpHomogeneousMatrix cextMo(0,0,3, 0,0,0);

    while(1) {
      robot.getPosition(wMc);
      cMo = wMc.inverse() * wMo;
      for (unsigned int i = 0 ; i < 4 ; i++) {
        point[i].track(cMo);
        std::cout << "p["<<i<<"]: " << p[i][0] << ", " << p[i][1] << ", " << p[i][2] << std::endl;
        //std::cout << "pd["<<i<<"]" << pd[i][0] << ", " << pd[i][1] << ", " << pd[i][2] << std::endl;
        vpFeatureBuilder::create(p[i], point[i]);
      }
      std::cout<<std::endl;
      vpColVector v = task.computeControlLaw();
      vpColVector e = task.getError(); 
      std::cout<<"e=:"<<e[0]<<","<<e[1]<<","<<e[2]<<","<<e[3]<<","<<e[4]<<","<<e[5]<<std::endl;

      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      vpDisplay::display(Iint) ;
      vpDisplay::display(Iext) ;
      display_trajectory(Iint, point, cMo, cam);

      vpServoDisplay::display(task, cam, Iint, vpColor::green, vpColor::red);

      externalview.display(Iext, cextMo, cMo, cam, vpColor::red, true);

      vpDisplay::flush(Iint);
      vpDisplay::flush(Iext);

      // A click to exit
      if (vpDisplay::getClick(Iint, false) || vpDisplay::getClick(Iext, false))
        break;

      vpTime::wait( robot.getSamplingTime() * 1000);
    }
    task.kill();
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}
