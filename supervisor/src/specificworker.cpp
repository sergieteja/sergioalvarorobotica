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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	innermodel = new InnerModel("/home/robocomp/robocomp/files/innermodel/simpleworld.xml");  
	tag.Init(innermodel);
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
	
	RoboCompDifferentialRobot::TBaseState bState;
	differentialrobot_proxy->getBaseState(bState);
	innermodel->updateTransformValues("base", bState.x,0,bState.z,0 ,bState.alpha,0);
	
	switch (estado)
			{	
				case State::SEARCH:	
					
					qDebug()<<"SEARH";
					qDebug()<<"Current en el search"<<current;
					
					if(tag.getID() == current)
					{
						differentialrobot_proxy->stopBase();
						gotopoint_proxy->go("", tag.getPose().x(), tag.getPose().z(), 0);
						estado = State::WAIT;
						qDebug()<<"De SEARH  --> wait";
					}
					qDebug()<<"Girando";
					differentialrobot_proxy->setSpeedBase(0,0.3);
					break;
				case State::WAIT:
					qDebug()<<"Entrando en wait"<<gotopoint_proxy->atTarget();
					
					if( gotopoint_proxy->atTarget() == true)
					{
						differentialrobot_proxy->stopBase();
						estado = State::SEARCH;
						qDebug()<<"De WAIT  --> SEARCH";
						current = (current+1)%4;
						qDebug()<<"Current"<<current;
						sleep(3);
					}
					else if( tag.hasChanged() && tag.getID() == current)
					{
						
						gotopoint_proxy->go("", tag.getPose().x(), tag.getPose().z(), 0);
					}
					
					break;
			}
}


////////////////////////////////////////////////
void SpecificWorker::newAprilTag(const tagsList& tags)
{
	qDebug()<<"LLegando las april tags" << tags.front().tx << tags.front().tz;
	tag.copy(tags.front().tx, tags.front().tz, tags.front().id);
	
}













