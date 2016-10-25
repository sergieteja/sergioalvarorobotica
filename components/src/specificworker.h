/*
 *    Copyright (C) 2016 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <qmat/QMatAll>

#include <complex>



class SpecificWorker : public GenericWorker
{
Q_OBJECT

struct Target{
  bool active =false;
  mutable QMutex m;
  QVec pose;
  void setActive(bool v){
    QMutexLocker ml(&m);
    active=v;
  }
  float ang;
  void copy(float x, float z){
    QMutexLocker ml(&m);
    pose[0]=x;
    pose[1]=z;    
  }
  QVec getPose(){
    QMutexLocker ml(&m);
    return pose;    
  }
};

// 

//Estructura de la maquina de estaddos.
;
enum class State {INIT,GOTO,BUG,STOP, IDLE};
State estado = State::INIT;

public:
	
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void setPick(const Pick &myPick);
	void gotoTarget(RoboCompLaser::TLaserData ldata);
	bool obstacle(RoboCompLaser::TLaserData ldata);
	void bug(RoboCompLaser::TLaserData ldata);
	bool targetAtSight(RoboCompLaser::TLaserData ldata);
	void stop();

public slots:
	void compute(); 	

private:
  Target target;
  InnerModel *innermodel;
	
};

#endif

