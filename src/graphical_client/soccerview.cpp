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
\file    soccerview.cpp
\brief   C++ Implementation: GLSoccerView
\author  Joydeep Biswas (C) 2011
*/
//========================================================================

#include "soccerview.h"

GLSoccerView::FieldDimensions::FieldDimensions()
{
  line_width = FieldConstantsRoboCup2009::line_width;
  field_length = 5000;//FieldConstantsRoboCup2009::field_length;
  field_width = 3500;//FieldConstantsRoboCup2009::field_width;
  boundary_width = FieldConstantsRoboCup2009::boundary_width;
  referee_width = FieldConstantsRoboCup2009::referee_width;
  goal_width = FieldConstantsRoboCup2009::goal_width;
  goal_depth = FieldConstantsRoboCup2009::goal_depth;
  goal_wall_width = FieldConstantsRoboCup2009::goal_wall_width;
  center_circle_radius = FieldConstantsRoboCup2009::center_circle_radius;
  defense_radius = FieldConstantsRoboCup2009::defense_radius;
  defense_stretch = FieldConstantsRoboCup2009::defense_stretch;
  free_kick_from_defense_dist = FieldConstantsRoboCup2009::free_kick_from_defense_dist;
  penalty_spot_from_field_line_dist = FieldConstantsRoboCup2009::penalty_spot_from_field_line_dist;
  penalty_line_from_spot_dist = FieldConstantsRoboCup2009::penalty_line_from_spot_dist;
}

GLSoccerView::GLSoccerView(QWidget* parent) : QGLWidget(QGLFormat ( QGL::DoubleBuffer | QGL::DepthBuffer | QGL::SampleBuffers),parent)
{ 
  viewScale = (fieldDim.field_length+fieldDim.boundary_width+fieldDim.referee_width)/sizeHint().width();
  viewScale = max(viewScale, (fieldDim.field_width+fieldDim.boundary_width+fieldDim.referee_width)/sizeHint().height());
  viewXOffset = viewYOffset = 0.0;
  setAutoFillBackground(false); //Do not let painter auto fill the widget's background: we'll do it manually through openGl
  connect(this, SIGNAL(postRedraw()), this, SLOT(redraw()));
  fieldLinesList = GL_INVALID_VALUE;
  blueRobotShape = GL_INVALID_VALUE;
  yellowRobotShape = GL_INVALID_VALUE;
  greyRobotShape = GL_INVALID_VALUE;
  blueCircleRobotShape = GL_INVALID_VALUE;
  yellowCircleRobotShape = GL_INVALID_VALUE;
  greyCircleRobotShape = GL_INVALID_VALUE;
  QFont RobotIDFont = this->font();
  RobotIDFont.setWeight(QFont::Bold);
  RobotIDFont.setPointSize(80);
  glText = GLText(RobotIDFont);
  tLastRedraw = 0;
  ball.set(0, 0);
}

void GLSoccerView::redraw()
{
  if(GetTimeSec()-tLastRedraw<MinRedrawInterval)
    return;
  graphicsMutex.lock();
  update();
  graphicsMutex.unlock();
  tLastRedraw = GetTimeSec();
}


void GLSoccerView::mousePressEvent(QMouseEvent* event)
{
  leftButton = event->buttons().testFlag(Qt::LeftButton);
  midButton = event->buttons().testFlag(Qt::MidButton);
  rightButton = event->buttons().testFlag(Qt::RightButton);
  bool shiftKey = event->modifiers().testFlag(Qt::ShiftModifier);
  bool ctrlKey = event->modifiers().testFlag(Qt::ControlModifier);
  
  if(leftButton)
    setCursor(Qt::ClosedHandCursor);
  if(midButton)
    setCursor(Qt::SizeVerCursor);
  if(leftButton || midButton){
    // Start Pan / Zoom
    mouseStartX = event->x();
    mouseStartY = event->y();
    postRedraw();
  }  
}

void GLSoccerView::mouseReleaseEvent(QMouseEvent* event)
{
  bool shiftKey = event->modifiers().testFlag(Qt::ShiftModifier);
  bool ctrlKey = event->modifiers().testFlag(Qt::ControlModifier);
  setCursor(Qt::ArrowCursor);
}

void GLSoccerView::mouseMoveEvent(QMouseEvent* event)
{
  static const bool debug = false;
  bool leftButton = event->buttons().testFlag(Qt::LeftButton);
  bool midButton = event->buttons().testFlag(Qt::MidButton);
  bool rightButton = event->buttons().testFlag(Qt::RightButton);
  
  if(debug) printf("MouseMove Event, Left:%d Mid:%d Right:%d\n", leftButton?1:0, midButton?1:0, rightButton?1:0);
  
  if(leftButton){
    //Pan
    viewXOffset -= viewScale*double(event->x() - mouseStartX);
    viewYOffset += viewScale*double(event->y() - mouseStartY);
    mouseStartX = event->x();
    mouseStartY = event->y();
    recomputeProjection();
    postRedraw();
  }else if(midButton){
    //Zoom
    double zoomRatio = double(event->y() - mouseStartY)/500.0;
    double oldScale = viewScale;
    viewScale = viewScale*(1.0+zoomRatio);
    recomputeProjection();
    mouseStartX = event->x();
    mouseStartY = event->y();
    postRedraw();
  }
}

void GLSoccerView::wheelEvent(QWheelEvent* event)
{
  static const bool debug = false;
  double zoomRatio = -double(event->delta())/1000.0;
  double oldScale = viewScale;
  viewScale = viewScale*(1.0+zoomRatio);
  recomputeProjection();
  if(debug) printf("Zoom: %5.3f\n",viewScale);
  postRedraw();
}

void GLSoccerView::keyPressEvent(QKeyEvent* event)
{
  static const bool debug = false;
  if(debug) printf("KeyPress: 0x%08X\n",event->key());
  if(event->key() == Qt::Key_Space)
    resetView();
  if(event->key() == Qt::Key_Escape)
    close();
}

void GLSoccerView::resetView()
{
  viewScale = (fieldDim.field_length+fieldDim.boundary_width+fieldDim.referee_width)/width();
  viewScale = max(viewScale, (fieldDim.field_width+fieldDim.boundary_width+fieldDim.referee_width)/height());
  viewXOffset = viewYOffset = 0.0;
  recomputeProjection();
  postRedraw();
}

void GLSoccerView::resizeEvent(QResizeEvent* event)
{
  QGLWidget::resizeEvent(event);
  redraw();
}

void GLSoccerView::recomputeProjection()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-0.5*viewScale*width()+viewXOffset, 0.5*viewScale*width()+viewXOffset, -0.5*viewScale*height()+viewYOffset, 0.5*viewScale*height()+viewYOffset, minZValue, maxZValue);
  glMatrixMode(GL_MODELVIEW);
}

void GLSoccerView::resizeGL(int width, int height)
{
  glViewport(0, 0, width, height);
  recomputeProjection();
}

void GLSoccerView::initializeGL()
{ 
  fieldLinesList = glGenLists(1);
  if(fieldLinesList==GL_INVALID_VALUE){
    printf("Unable to create display list!\n");
    exit(1);
  }
  glNewList(fieldLinesList, GL_COMPILE);
  drawFieldLines(fieldDim);
  glEndList();
  
  
  blueRobotShape = glGenLists(1);
  if(blueRobotShape==GL_INVALID_VALUE){
    printf("Unable to create display list!\n");
    exit(1);
  }
  glNewList(blueRobotShape, GL_COMPILE);
  drawRobot(teamBlue,true,false);
  glEndList();
  
  yellowRobotShape = glGenLists(1);
  if(yellowRobotShape==GL_INVALID_VALUE){
    printf("Unable to create display list!\n");
    exit(1);
  }
  glNewList(yellowRobotShape, GL_COMPILE);
  drawRobot(teamYellow,true,false);
  glEndList();
  
  greyRobotShape = glGenLists(1);
  if(greyRobotShape==GL_INVALID_VALUE){
    printf("Unable to create display list!\n");
    exit(1);
  }
  glNewList(greyRobotShape, GL_COMPILE);
  drawRobot(teamUnknown,true,false);
  glEndList();
  
  blueCircleRobotShape = glGenLists(1);
  if(blueRobotShape==GL_INVALID_VALUE){
    printf("Unable to create display list!\n");
    exit(1);
  }
  glNewList(blueCircleRobotShape, GL_COMPILE);
  drawRobot(teamBlue,false,false);
  glEndList();
  
  yellowCircleRobotShape = glGenLists(1);
  if(yellowRobotShape==GL_INVALID_VALUE){
    printf("Unable to create display list!\n");
    exit(1);
  }
  glNewList(yellowCircleRobotShape, GL_COMPILE);
  drawRobot(teamYellow,false,false);
  glEndList();
  
  greyCircleRobotShape = glGenLists(1);
  if(greyRobotShape==GL_INVALID_VALUE){
    printf("Unable to create display list!\n");
    exit(1);
  }
  glNewList(greyCircleRobotShape, GL_COMPILE);
  drawRobot(teamUnknown,false,false);
  glEndList();
}


void GLSoccerView::vectorTextTest()
{   
  #define TextTest(loc,angle,size,str,halign,valign) \
  {glText.drawString((loc),angle,size,str,halign,valign); \
  vector2d l1,l2; \
  l1.heading(M_PI/180.0*angle); \
  l1 = loc+l1*size*(glText.getWidth(str)); \
  l2.heading(M_PI/180.0*angle); \
  l2 = loc-l2*size*(glText.getWidth(str)); \
  glBegin(GL_LINES); \
  glVertex3d(l1.x,l1.y,9); \
  glVertex3d(l2.x,l2.y,9); \
  glEnd();}
  
  glColor3d(1,1,1);
  TextTest(vector2d(1,1)*353.6,45,500,"123agdo0",GLText::LeftAligned,GLText::MedianAligned)
  TextTest(vector2d(fieldDim.field_length*0.5,0),0,500,"123agdo0",GLText::RightAligned,GLText::BottomAligned)
  TextTest(vector2d(0,-fieldDim.field_width*0.5),0,500,"123agdo0",GLText::CenterAligned,GLText::TopAligned)
  TextTest(vector2d(-fieldDim.field_length*0.5,0),0,500,"1\ub023agdo0",GLText::CenterAligned,GLText::MiddleAligned)
}

void GLSoccerView::paintEvent(QPaintEvent* event)
{
  graphicsMutex.lock();
  makeCurrent();
  glClearColor(FIELD_COLOR);
  glShadeModel(GL_SMOOTH);
  glDisable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_MULTISAMPLE);
  
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glCallList(fieldLinesList);
  drawRobots();
  drawBalls();
  drawPoints();
  draw_target_point();
  //vectorTextTest();
  glPopMatrix();
  swapBuffers();

  // drawPoints();
  graphicsMutex.unlock();
}

void GLSoccerView::drawVector(vector2d locOrigen, vector2d locDestino, double r, double g, double b)
{
	glBegin(GL_LINES);
  glColor3d(r,g,b);
	glVertex3d(locOrigen.x,locOrigen.y,RobotZ);
	glVertex3d(locDestino.x,locDestino.y,RobotZ);
	glEnd();
}

void GLSoccerView::drawVector(vector2d locOrigen, vector2d locDestino)
{
  glBegin(GL_LINES);
  // glColor3d(r,g,b)
  glVertex3d(locOrigen.x,locOrigen.y,RobotZ);
  glVertex3d(locDestino.x,locDestino.y,RobotZ);
  glEnd();
}


void GLSoccerView::drawQuad(vector2d loc1, vector2d loc2, double z)
{
  glBegin(GL_QUADS);
  glVertex3d(loc1.x,loc1.y,z);
  glVertex3d(loc2.x,loc1.y,z);
  glVertex3d(loc2.x,loc2.y,z);
  glVertex3d(loc1.x,loc2.y,z);
  glEnd();
}

void GLSoccerView::drawArc(vector2d loc, double r1, double r2, double theta1, double theta2, double z, double dTheta)
{
  static const double tesselation = 1.0;
  if(dTheta<0){
    dTheta = tesselation/r2;
  }
  glBegin(GL_QUAD_STRIP);
  for(double theta=theta1; theta<theta2; theta+=dTheta){
    double c1 = cos(theta), s1 = sin(theta);
    glVertex3d(r2*c1+loc.x,r2*s1+loc.y,z);
    glVertex3d(r1*c1+loc.x,r1*s1+loc.y,z);
  }
  double c1 = cos(theta2), s1 = sin(theta2);
  glVertex3d(r2*c1+loc.x,r2*s1+loc.y,z);
  glVertex3d(r1*c1+loc.x,r1*s1+loc.y,z);
  glEnd();
}

void GLSoccerView::drawRobot(int team, bool hasAngle, bool useDisplayLists)
{
  if(useDisplayLists){
    switch ( team ){
      case teamBlue:{
        if(hasAngle)
          glCallList(blueRobotShape);
        else
          glCallList(blueCircleRobotShape);
        break;
      }
      case teamYellow:{
        if(hasAngle)
          glCallList(yellowRobotShape);
        else
          glCallList(yellowCircleRobotShape);
        break;
      }
      default:{
        if(hasAngle)
          glCallList(greyRobotShape);
        else
          glCallList(greyCircleRobotShape);
        break;
      }
    }
    return;
  }
  
  switch ( team ){
    case teamBlue:{
      glColor3d(0.2549, 0.4941, 1.0);
      break;
    }
    case teamYellow:{
      glColor3d(1.0, 0.9529, 0.2431);
      break;
    }
    default:{
      glColor3d(0.5882,0.5882,0.5882);
      break;
    }
  }
  double theta1 = hasAngle?RAD(40):0.0;
  double theta2 = 2.0*M_PI - theta1;
  drawArc(0,0,0,90,theta1, theta2, RobotZ);
  if(hasAngle){
    glBegin(GL_TRIANGLES);
    glVertex3d(0,0,RobotZ);
    glVertex3d(90.0*cos(theta1),90.0*sin(theta1),RobotZ);
    glVertex3d(90.0*cos(theta2),90.0*sin(theta2),RobotZ);
    glEnd();
  }
  
  switch ( team ){
    case teamBlue:{
      glColor3d(0.0706, 0.2314, 0.6275);
      break;
    }
    case teamYellow:{
      glColor3d(0.8, 0.6157, 0.0);
      break;
    }
    default:{
      glColor3d(0.2745,0.2745,0.2745);
      break;
    }
  }
  drawArc(0,0,80,90,theta1, theta2, RobotZ+0.01);
  if(hasAngle)
    drawQuad(90.0*cos(theta1)-10,90.0*sin(theta1), 90.0*cos(theta2),90.0*sin(theta2),RobotZ+0.01);
}

void GLSoccerView::drawRobot(vector2d loc, double theta, double conf, int robotID, int team, bool hasAngle, vector3d vel)
{
  glPushMatrix();
  glLoadIdentity();
  glTranslated(loc.x,loc.y,0);
  switch ( team ){
    case teamBlue:{
      glColor3d(0.2549, 0.4941, 1.0);
      break;
    }
    case teamYellow:{
      glColor3d(1.0, 0.9529, 0.2431);
      break;
    }
    default:{
      glColor3d(0.5882,0.5882,0.5882);
      break;
    }
  }
  drawQuad(-90,130,-90.0+180.0*conf,160,RobotZ);
  glColor3d(0.0,0.0,0.0);
  char buf[1024];
  if(robotID!=unknownRobotID)
    snprintf(buf,1023,"%X",robotID);
  else
    snprintf(buf,1023,"?");
  glText.drawString(loc,0,100,buf,GLText::CenterAligned,GLText::MiddleAligned);
  switch ( team ){
    case teamBlue:{
      glColor3d(0.0706, 0.2314, 0.6275);
      break;
    }
    case teamYellow:{
      glColor3d(0.8, 0.6157, 0.0);
      break;
    }
    default:{
      glColor3d(0.2745,0.2745,0.2745);
      break;
    }
  }
  drawQuad(-96,124,96.0,130,RobotZ+0.01);
  drawQuad(-96,124,-90.0,166,RobotZ+0.01);
  drawQuad(-96,160,96.0,166,RobotZ+0.01);
  drawQuad(90,124,96.0,166,RobotZ+0.01);
  
  glRotated(theta,0,0,1.0);
  drawRobot(team, hasAngle, true);
  glPopMatrix();

  vector2d a, b;
  a.set(loc.x,loc.y);
  b.set(vel.x,vel.y);
  drawVector(a,b);
}

void GLSoccerView::drawFieldLines(FieldDimensions& dimensions)
{
  glColor4f(FIELD_LINES_COLOR);
  
  //Field boundary lines
  drawQuad(-0.5*dimensions.field_length,-0.5*dimensions.field_width, 0.5*dimensions.field_length,-0.5*dimensions.field_width+dimensions.line_width,FieldZ);
  drawQuad(-0.5*dimensions.field_length,0.5*dimensions.field_width-dimensions.line_width, 0.5*dimensions.field_length,0.5*dimensions.field_width,FieldZ);
  drawQuad(-0.5*dimensions.field_length,-0.5*dimensions.field_width, -0.5*dimensions.field_length+dimensions.line_width,0.5*dimensions.field_width,FieldZ);
  drawQuad(0.5*dimensions.field_length-dimensions.line_width,-0.5*dimensions.field_width, 0.5*dimensions.field_length,0.5*dimensions.field_width,FieldZ);
  
  //Field Mid Line and Circle
  drawQuad(-0.5*dimensions.line_width,-0.5*dimensions.field_width, 0.5*dimensions.line_width,0.5*dimensions.field_width,FieldZ);
  drawArc(0.0,0.0, dimensions.center_circle_radius-0.5*dimensions.line_width, dimensions.center_circle_radius+0.5*dimensions.line_width, -M_PI, M_PI,FieldZ);
  
  //Goals
  drawQuad(-0.5*dimensions.field_length-dimensions.goal_depth-dimensions.goal_wall_width, -0.5*dimensions.goal_width-dimensions.goal_wall_width,
           -0.5*dimensions.field_length, -0.5*dimensions.goal_width,FieldZ);
  drawQuad(-0.5*dimensions.field_length-dimensions.goal_depth-dimensions.goal_wall_width, 0.5*dimensions.goal_width,
           -0.5*dimensions.field_length, 0.5*dimensions.goal_width+dimensions.goal_wall_width,FieldZ);
  drawQuad(-0.5*dimensions.field_length-dimensions.goal_depth-dimensions.goal_wall_width, -0.5*dimensions.goal_width-dimensions.goal_wall_width,
           -0.5*dimensions.field_length-dimensions.goal_depth, 0.5*dimensions.goal_width+dimensions.goal_wall_width,FieldZ);
  
  drawQuad(0.5*dimensions.field_length, -0.5*dimensions.goal_width-dimensions.goal_wall_width,
           0.5*dimensions.field_length+dimensions.goal_depth+dimensions.goal_wall_width, -0.5*dimensions.goal_width,FieldZ);
  drawQuad(0.5*dimensions.field_length, 0.5*dimensions.goal_width,
           0.5*dimensions.field_length+dimensions.goal_depth+dimensions.goal_wall_width, 0.5*dimensions.goal_width+dimensions.goal_wall_width,FieldZ);
  drawQuad(0.5*dimensions.field_length+dimensions.goal_depth, -0.5*dimensions.goal_width-dimensions.goal_wall_width,
           0.5*dimensions.field_length+dimensions.goal_depth+dimensions.goal_wall_width, 0.5*dimensions.goal_width+dimensions.goal_wall_width,FieldZ);

  //Defense Areas
  drawArc(-0.5*dimensions.field_length,0.5*dimensions.defense_stretch,dimensions.defense_radius-dimensions.line_width,dimensions.defense_radius,0, M_PI_2,FieldZ);
  drawArc(-0.5*dimensions.field_length,-0.5*dimensions.defense_stretch,dimensions.defense_radius-dimensions.line_width,dimensions.defense_radius,-M_PI_2, 0,FieldZ);
  drawQuad(-0.5*dimensions.field_length+dimensions.defense_radius-dimensions.line_width,-0.5*dimensions.defense_stretch,
            -0.5*dimensions.field_length+dimensions.defense_radius,0.5*dimensions.defense_stretch,FieldZ);
  drawArc(0.5*dimensions.field_length,0.5*dimensions.defense_stretch,dimensions.defense_radius-dimensions.line_width,dimensions.defense_radius,M_PI_2, M_PI,FieldZ);
  drawArc(0.5*dimensions.field_length,-0.5*dimensions.defense_stretch,dimensions.defense_radius-dimensions.line_width,dimensions.defense_radius,-M_PI, -M_PI_2,FieldZ);
  drawQuad(0.5*dimensions.field_length-dimensions.defense_radius,-0.5*dimensions.defense_stretch,
           0.5*dimensions.field_length-dimensions.defense_radius+dimensions.line_width,0.5*dimensions.defense_stretch,FieldZ);

  //Penalty marks
  drawArc(-0.5*dimensions.field_length+dimensions.penalty_spot_from_field_line_dist,0,0,10,-M_PI,M_PI,FieldZ);
  drawArc(0.5*dimensions.field_length-dimensions.penalty_spot_from_field_line_dist,0,0,10,-M_PI,M_PI,FieldZ);
  
}

void GLSoccerView::drawBall(vector2d loc)
{
  glColor3d(1.0,0.5059,0.0);
  drawArc(loc,0,16,-M_PI,M_PI,BallZ);
  glColor3d(0.8706,0.3490,0.0);
  drawArc(loc,15,21,-M_PI,M_PI,BallZ);
  
}

void GLSoccerView::drawBalls()
{
  drawBall(ball);
}

void GLSoccerView::drawRobots()
{
  for(int i=0; i<robots.size(); i++){
    Robot r = robots[i];
    drawRobot(r.loc,r.angle,r.conf,r.id,r.team,r.hasAngle,r.vel);
  }
}

void GLSoccerView::updateBall(const vector2d& detection)
{
	graphicsMutex.lock();
  
	ball.set(detection.x, detection.y);

	graphicsMutex.unlock();
	postRedraw();
}

void GLSoccerView::updateRobot(const Robot& robot)
{
	graphicsMutex.lock();

	int ind = robots.indexOf(robot);
	if (ind != -1)
		robots.remove(ind);
	robots.append(robot);

  graphicsMutex.unlock();
  postRedraw();
}

void GLSoccerView::updateRobotTraj(const Robot& robot)
{
        int ind = robots.indexOf(robot);
        if (ind != -1){
              Robot newRobot;

              newRobot.id = robot.id;
              newRobot.team = robot.team;
              newRobot.vel.set(robot.vel.x, robot.vel.y, robot.vel.z);

              newRobot.conf = 0.0;
              newRobot.loc.set(robots[ind].loc.x, robots[ind].loc.y);
              newRobot.angle = robots[ind].angle;
              updateRobot(newRobot);
      }
}

void GLSoccerView::drawPoint(const vector2d loc, double r, double g, double b, double rad, double z)
{
  glColor3d(r,g,b);
  drawArc(loc,0, rad,-M_PI,M_PI,z);
  // glColor3d(0.8706,0.3490,0.0);
  // drawArc(loc,15,21,-M_PI,M_PI,BallZ);
  
}
void GLSoccerView::drawPoints()
{
  // printf("points: %d\n", points.size());
  vector2d v1;
  vector2d v2;
  vector2d v_theta;

  if (target_points.size() > 0)
  {
    v1.x = target_points[0].x;
    v1.y = target_points[0].y;
    v_theta.x = 100.0 * sin(target_points[0].z) + v1.x;
    v_theta.y = 100.0 * cos(target_points[0].z) + v1.y;
    // cout << "v_t: (" << v_theta.x << ", " << v_theta.y << ")" << endl;
    drawPoint(v1, .08, 0.8, 0.89, 32, 4);
    drawVector(v1, v_theta, 0.8, 0.08, 0.89);
  }
  for (int i = 1; i < target_points.size(); ++i)
  { 
    v2.x = target_points[i].x;
    v2.y = target_points[i].y;
    v_theta.x = 100.0 * sin(target_points[i].z) + v2.x;
    v_theta.y = 100.0 * cos(target_points[i].z) + v2.y;
    drawPoint(v2, .08, 0.8, 0.89, 32,4);
    drawVector(v1, v2, 0.08, 0.8, 0.89);
    drawVector(v2, v_theta, 0.8, 0.08, 0.89);

    v1.x = v2.x;
    v1.y = v2.y;
  }

  // for (int i = 0; i < target_points.size(); ++i)
  // {
  //   drawPoint(target_points[i], .08, 0.8, 0.89, 32,4);
  // }

  for (int i = 0; i < target_points_debug.size(); ++i)
  {
    drawPoint(target_points_debug[i], 0.2588, 0.95686, 0.5961, 16, 6);
    

  }
  for (int i = 0; i < points.size(); ++i)
  {
    // printf("( %f , %f )", points[i].x, points[i].y);
    drawPoint(points[i], 1.0, 0.0, 0.0, 16,5);
  }
}
void GLSoccerView::updatePoints(QVector <vector2d> & p)
{
  // points.data = p.data
  points = p;
  // printf("SIZE: %d" , points.size());
  graphicsMutex.lock();
  
  // for (int i = 0; i < points.size(); ++i)
  // {
  //   points[i].set(point.x, point.y);
  // }
  // ball.set(detection.x, detection.y);

  graphicsMutex.unlock();
  postRedraw();
}


void GLSoccerView::update_target_points(QVector <vector3d> & p)
{
  // points.data = p.data
  target_points = p;
  // printf("SIZE: %d" , points.size());
  graphicsMutex.lock();
  
  // for (int i = 0; i < points.size(); ++i)
  // {
  //   points[i].set(point.x, point.y);
  // }
  // ball.set(detection.x, detection.y);

  graphicsMutex.unlock();
  postRedraw();
}

void GLSoccerView::draw_target_point()
{
  glColor3d(0.0,0.38,1.0);
  drawArc(target_point,0,30,-M_PI,M_PI,BallZ);
  // glColor3d(0.8706,0.3490,0.0);
  // drawArc(loc,15,21,-M_PI,M_PI,BallZ);
  glColor3d(1, 0, 0.86);
  drawArc(dest_point, 0, .32*200, -M_PI, M_PI, 0 );
  
}

void GLSoccerView::update_dest_point(double x, double y)
{
  graphicsMutex.lock();
  dest_point.set(x, y);
  graphicsMutex.unlock();
  postRedraw();
}

void GLSoccerView::update_target_point(double x, double y)
{
  graphicsMutex.lock();
  target_point.set(x, y);
  graphicsMutex.unlock();
  postRedraw();
}

void GLSoccerView::update_target_point(QVector <vector2d> & p)
{
  graphicsMutex.lock();
  target_points_debug = p;
  // target_points_debug.add(set(x, y));
  graphicsMutex.unlock();
  postRedraw();

}
void GLSoccerView::draw_dest_point()
{
  glColor3d(1, 0, 0.86);
  drawArc(dest_point, 0, .32*200, -M_PI, M_PI );
}

