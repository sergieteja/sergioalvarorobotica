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
	timer.start(Period);
	return true;
}


void SpecificWorker::compute()
{  
    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
        //std::sort( ldata.begin()+20, ldata.end()-20, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
		RoboCompDifferentialRobot::TBaseState bState;
		differentialrobot_proxy->getBaseState(bState);
    
		switch (estado)
		{
	    
			case State::INIT:
			
				if(target.active)
					estado = State::GOTO;
			break;
				
			case State::GOTO:	
				gotoTarget(ldata);
				break;
		
			
			case State::BUG:
				
				break;

			case State::STOP:
				
				SpecificWorker::stop();
				
				break;
		}
		
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}		  
}

/////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void SpecificWorker::gotoTarget(RoboCompLaser::TLaserData ldata)
{
	QVec tr = innermodel->transform("base", target.getPose(), "world");
	float ang = atan2(tr.x(), tr.z());
	float dist = tr.norm2();
 	
	if( dist < 50) 
	{
		//estado = State::IDLE;
		target.setActive(false);
		differentialrobot_proxy->stopBase();
		return;
	}
 
 
 	//comprobar si hay obstaculo
 	if( obstacle(ldata)){
		
		estado = State::BUG;
	}
    //std::sort( ldata.begin()+20, ldata.end()-20, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
	
 
	//controlador
	float vadv = dist;
	float vrot= ang;
	if( fabs(vrot) > 0.05)
		vadv = 0;
	differentialrobot_proxy->setSpeedBase(vadv,vrot);
}

//Falta probar

/**
void SpecificWorker::irA(RoboCompLaser::TLaserData ldata)
{
   const float threshold = 420; //millimeters
   float rot = 0.6;  //rads per second
   if( (ldata.data()+20)->dist < threshold){
     
     differentialrobot_proxy->setSpeedBase(10, rot);
     usleep(rand()%(1500000-100000 + 1) + 100000);  //random wait between 1.5s and 0.1sec
     estado=State::STOP;
     
   } else
    {
	differentialrobot_proxy->setSpeedBase(400, 0); 
    }
}

*/

bool SpecificWorker::obstacle(RoboCompLaser::TLaserData ldata)
{
	const float threshold = 420;
	std::sort( ldata.begin()+20, ldata.end()-20, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
	if( (ldata.data()+20)->dist < threshold){
		return true;
	}
	
 return false;  
}

void SpecificWorker::stop()
{
 
  
}



//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

void SpecificWorker::setPick(const Pick& myPick)
{ 
   qDebug() << myPick.x <<":"<< myPick.z ;
   target.copy(myPick.x,myPick.z);
   target.setActive(true); 
}