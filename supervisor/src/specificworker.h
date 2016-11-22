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

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void newAprilTag(const tagsList &tags);
	enum class State  { SEARCH, WAIT} ;  
	State estado = State::SEARCH;
	InnerModel *innermodel;
	int current =0;

public slots:
	void compute(); 	

private:
	
	struct Target
		{
			bool active =false;
			mutable QMutex m;
			QVec pose = QVec::zeros(3);
			QVec poseAnt = QVec::zeros(3);
			int ident;
			InnerModel *inner;
			
			void Init (InnerModel*innermodel){
				inner =innermodel;
			}

			void setActive(bool v)
			{
				QMutexLocker ml(&m);
				active=v;
			}
			float ang;
			
			int getID()
			{
				QMutexLocker ml(&m);
				return ident;
			}
			
			void copy(float x, float z, int id)
			{
				QMutexLocker ml(&m); 
				//qDebug()<<"Posicion antes"<< x << z<<id;
				pose = inner->transform("world",QVec::vec3(x,0,z) ,"rgbd");
				//qDebug()<<"Posicion despues" <<pose.x()<<pose.z();
				ident=id;
				
			}
			bool hasChanged()
			{
				 if( (pose-poseAnt).norm2() > 100)
				 {
					 poseAnt = pose;
					 return true;
				 }
				 return false;
			}
			
			QVec getPose()
			{
				QMutexLocker ml(&m);
				return pose;    
			}
		};
		Target tag;
};


#endif

