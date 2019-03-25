//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    soccerview.h
\brief   C++ Interface: GLSoccerView
\author  Joydeep Biswas (C) 2011
*/
//========================================================================

#include <QMouseEvent>
#include <QWidget>
//#include <QGLWidget>
#include <QtOpenGL/QGLWidget>
#include <QMutex>
#include <QVector>
// #include <GL/glu.h>
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <cstdio>
#include "timer.h"
#include "geometry.h"
#include "util.h"
#include "gltext.h"
#include "field_default_constants.h"
#include "structures.h"

#ifdef __APPLE__
#  include <OpenGL/gl.h>
#  include <OpenGL/glu.h>
#  include <GLUT/glut.h>
#else
#  include <GL/gl.h>
#  include <GL/glu.h>
#  include <GL/freeglut.h>
#endif


#ifndef SOCCERVIEW_H
#define SOCCERVIEW_H

using namespace std;

#define FIELD_COLOR 0.0,0.5686,0.0980,1.0
#define FIELD_LINES_COLOR 1.0,1.0,1.0,1.0

class GLSoccerView : public QGLWidget{
  Q_OBJECT
  
public:
  struct FieldDimensions{
    /// Width of field lines. Check SSL Rules for conventions regarding line widths and areas enclosed.
    double line_width;
    /// Length of the field
    double field_length;
    /// Width of the field
    double field_width;
    /// Width of the boundary outside the field for robots to use if necessary
    double boundary_width;
    /// Width of the boundary outside the field for the referees to use only
    double referee_width;
    /// Width of Goal box
    double goal_width;
    /// Depth of Goal box (extending outside the field)
    double goal_depth;
    /// Width of the walls outlining the goal box
    double goal_wall_width;
    /// Radius of the circle at the center of the field
    double center_circle_radius;
   // /// Radius of the Defense area quarter circles
    double defense_radius;
    /// Length of the line joining the quarter circles of the defense area
    double defense_stretch;
    /// Minimum clearance robots have to keep from defense area during free kicks
    double free_kick_from_defense_dist;
    /// Distance of penalty kick location from the field line (goal line) (Law 14)
    double penalty_spot_from_field_line_dist;
    /// Distance from the penalty mark that all other team members have to stay begind during a penalty kick (Law 14)
    double penalty_line_from_spot_dist;
    
    /// Load SSL 2009 Field Dimensions
    FieldDimensions();
  };

private:
  static constexpr double minZValue = -10;
  static constexpr double maxZValue = 10;
  static constexpr double FieldZ = 1.0;
  static constexpr double RobotZ = 2.0;
  static constexpr double BallZ = 3.0;
  static constexpr int PreferedWidth = 1024;
  static constexpr int PreferedHeight = 768;
  static constexpr double MinRedrawInterval = 0.016; ///Minimum time between graphics updates (limits the fps)
  static constexpr int unknownRobotID = -1;
  
  QVector <Robot> robots;
  QVector <int> robotsids;
  QVector <QVector<vector2d> > balls;
  vector2d ball;
  vector2d target_point;
  vector2d dest_point;
  QVector <vector2d> points;
  QVector <vector2d> target_points_debug;
  QVector <vector3d> target_points;
  QMutex graphicsMutex;
  GLText glText;
  
  GLuint fieldLinesList;
  GLuint blueRobotShape;
  GLuint yellowRobotShape;
  GLuint greyRobotShape;
  GLuint blueCircleRobotShape;
  GLuint yellowCircleRobotShape;
  GLuint greyCircleRobotShape;
  
  double viewScale; /// Ratio of world space to screen space coordinates
  double viewXOffset;
  double viewYOffset;
  
  bool leftButton;
  bool midButton;
  bool rightButton;
  int mouseStartX;
  int mouseStartY;
  
  double tLastRedraw;
  
  FieldDimensions fieldDim;
  
private:
  void drawFieldLines(FieldDimensions &dimensions);
  void drawRobots();
  void drawBalls();
  void drawVector(vector2d locOrigen, vector2d locDestino);
  void drawVector(vector2d locOrigen, vector2d locDestino, double r, double g, double b);
  void drawQuad(vector2d loc1, vector2d loc2, double z=0.0);
  void drawQuad(double x1, double y1, double x2, double y2, double z=0.0){drawQuad(vector2d(x1,y1),vector2d(x2,y2),z);}
  void drawArc(vector2d loc, double r1, double r2, double theta1, double theta2, double z=0.0, double dTheta = -1);
  void drawArc(double x, double y, double r1, double r2, double theta1, double theta2, double z=0.0, double dTheta = -1){drawArc(vector2d(x,y),r1,r2,theta1,theta2,z,dTheta);}
  void recomputeProjection();
  void drawRobot(vector2d loc, double theta, double conf, int robotID, int team, bool hasAngle, vector3d vel);
  void drawRobot(int team, bool hasAngle, bool useDisplayLists);
  int UpdateBalls ( QVector<QPointF> &_balls, int cameraID );
  void drawBall(vector2d loc);
  void vectorTextTest();
  void drawPoint(vector2d loc, double r, double g, double b, double rad, double z);
  void drawPoints();
  void draw_target_point();
  void draw_dest_point();

protected:
  void paintEvent ( QPaintEvent * event );
  void wheelEvent ( QWheelEvent * event );
  void mouseMoveEvent ( QMouseEvent * event );
  void mousePressEvent( QMouseEvent * event );
  void mouseReleaseEvent( QMouseEvent * event );
  void keyPressEvent(QKeyEvent* event);
  void resizeEvent ( QResizeEvent * event );
  void initializeGL();
  void resizeGL(int width, int height);
  QSize sizeHint () const {return QSize(PreferedWidth,PreferedHeight);} 
  
public: 
  GLSoccerView(QWidget *parent = 0);
  void updateRobot(const Robot &robot);
  void updateRobotTraj(const Robot &robot);
  void updateBall(const vector2d &detection);
  void updatePoints(QVector <vector2d> & p);
  void update_target_points(QVector <vector3d> & p);
  void update_target_point(QVector <vector2d> & p);
  void update_target_point(double x, double y);
  void update_dest_point(double x, double y);
  
public slots:
  void resetView();
private slots:
  void redraw();
signals:
  void postRedraw();
};

#endif
