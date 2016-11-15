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
		RoboCompDifferentialRobot::TBaseState bState;
		differentialrobot_proxy->getBaseState(bState);
		innermodel->updateTransformValues("base", bState.x,0,bState.z,0 ,bState.alpha,0);
		QVec ini;
			switch (estado)
			{
				case State::INIT:
					if(target.active){
						qDebug()<<" INIT a GOTO";
						ini = QVec::vec3(bState.x, 0, bState.z);
						linea = QLine2D( ini, target.getPose() );
						estado=State::GOTO;
					}
				break;
					
				case State::GOTO:	
					gotoTarget(ldata);
					break;
			
				case State::BUG:
					bug( ldata,bState );
					break;
					
				case State::INITBUG:	
					initbug( ldata,bState );
					break;

				case State::STOP:
					stopLocal();
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
	
	QVec tr = innermodel->transform ( "base",target.getPose(),"world" );

	float angle = atan2 ( tr.x(),tr.z() );
	float distance = tr.norm2();
	//Comprobar si esta en el objetivo
	if( distance < 280) 
	{
		target.setActive(false);
		qDebug() << "FINALIZADO: GOTO A INIT";
		estado = State::INIT;
		differentialrobot_proxy->stopBase();
		return;
	}
	
	//comprobar si hay obstaculo
	if( obstacle(ldata)){
		estado = State::INITBUG;
		qDebug()<<"Hay obstaculo -- INITBUG";
		return;
	}
 
    //std::sort( ldata.begin()+20, ldata.end()-20, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
	
	//controlador
	if ( abs ( angle) > 0.05 )
		distance = 0;
	if( distance > 300) distance = 300;
	
	try
	{

		differentialrobot_proxy->setSpeedBase(distance,angle);
		
	}catch ( const Ice::Exception &ex ) {  std::cout << ex << std::endl; }
	
	
}




void SpecificWorker::bug(RoboCompLaser::TLaserData ldata,const TBaseState& bState )
{
	
	const float alpha = log ( 0.1 ) /log ( 0.3 ); //amortigua /corte
	float dist = obstacleLeft(ldata);
	float diffToline = distanceToLine(bState);
	
		if(targetAtSight(ldata)){
			estado  =  State::GOTO;
			qDebug() << "Target visible: desde BUG a GOTO";
			return;
		}
		
			//Check if close to the line and distance is reducing
		if (distanciaAnterior < 100 and diffToline < 0)
		{
			estado = State::GOTO;
			qDebug() << "Cruzando la linea: desde BUG a GOTO";
			return;
		}
		
		//Comprobar si hay obstaculo
		if( obstacle(ldata))
		{
			estado = State::INITBUG;
			qDebug()<<"Hay obstaculo -- INITBUG";
			return;
			
		}
		
		/*
		float d = 999999;
		for(int i=10; i<15; i++)
			if(ldata[i].dist < d)
				d = ldata[i].dist;
		
		//float d = ldata[15].dist; 
		if (d>160)
		{
			vrot = (d * 1./500 )- 0.5;
			//differentialrobot_proxy->stopBase();
			//usleep(1000000);  //random wait between 1.5s and 0.1sec
			if(vrot> 0.5){
				vrot = 0.5;
			}
			qDebug()<< "vrot pos: " << vrot;
		}
		if (d<140){
			vrot= -((d * 1/1000) -0.5);
			qDebug()<< "vrot neg: " << vrot;
			
		}
		*/
		
		float k=0.1;  // pendiente de la sigmoide
		float vrot =  -((1./(1. + exp(-k*(dist - 450.))))-1./2.);		//sigmoide para meter vrot entre -0.5 y 0.5. La k ajusta la pendiente.
		float vadv = 350 * exp ( - ( fabs ( vrot ) * alpha ) ); 		//gaussiana para amortiguar la vel. de avance en funcion de vrot
		qDebug() << vrot << vadv;
		//vrot *= 0.3;
		differentialrobot_proxy->setSpeedBase ( vadv ,vrot );
}
void SpecificWorker::initbug(const RoboCompLaser::TLaserData &ldata,const TBaseState& bState )
{
	QVec posi = QVec::vec3(bState.x, 0., bState.z);
	distanciaAnterior = fabs(linea.perpendicularDistanceToPoint(posi));
	
	if( obstacle(ldata) == false)
		{
			estado = State::BUG;
			qDebug()<<"No Hay obstaculo -- INITBUG---> BUG";
			return;
		}
	
	 try
		{
			differentialrobot_proxy->setSpeedBase(0, 0.3);
		}
		catch ( const Ice::Exception &ex ) {  std::cout << ex << std::endl; }
		
}	




bool SpecificWorker::targetAtSight(const RoboCompLaser::TLaserData &ldata)

{
	QPolygon poly;
	for ( auto l: ldata )
	{
		QVec r = innermodel->laserTo ( "world","laser",l.dist,l.angle );
		QPoint p ( r.x(),r.z() );
		poly << p;
	}
	QVec targetInRobot = innermodel->transform("base", target.getPose(), "world");
	float dist = targetInRobot.norm2();
	int veces = int(dist / 200);  //number of times the robot semilength fits in the robot-to-target distance
	float landa = 1./veces;
	
	QList<QPoint> points;
	points << QPoint(target.getPose().x(),target.getPose().z());  //Add target
	
	//Add points along lateral lines of robot
	for (float i=landa; i<= 1.; i+=landa)
	{
		QVec point = targetInRobot*(T)landa;
		QVec pointW = innermodel->transform("world", point ,"base");
		points << QPoint(pointW.x(), pointW.z());
		
		pointW = innermodel->transform("world", point - QVec::vec3(200,0,0), "base");
		points << QPoint(pointW.x(), pointW.z());
		
		pointW = innermodel->transform("world", point + QVec::vec3(200,0,0), "base");
		points << QPoint(pointW.x(), pointW.z());
		
	}
	foreach( QPoint p, points)
	{
		if( poly.containsPoint(p , Qt::OddEvenFill) == false)
			return false;
	}
	return true;
}

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
	const int offset = 30;
	const int minDist = 280;
	
	//sort laser data from small to large distances using a lambda function.
	std::sort ( ldata.begin() + offset, ldata.end()- offset, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ){	return a.dist < b.dist;});
	return ( ldata[offset].dist < minDist );
}

void SpecificWorker::stopLocal()
{
  qDebug("Parado");
  differentialrobot_proxy->setSpeedBase(0,0); 
  estado=State::IDLE;
  qDebug()<<"PARADO -- IDLE";
  target.setActive(false);
}

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////


float SpecificWorker::distanceToLine(const TBaseState& bState)
{
	QVec posi = QVec::vec3(bState.x, 0., bState.z);
	float distanciaEnPunto = fabs(linea.perpendicularDistanceToPoint(posi));
	float diff = distanciaEnPunto - distanciaAnterior;
	distanciaAnterior = distanciaEnPunto;
	return diff;
}  

float SpecificWorker::obstacleLeft(const TLaserData& tlaser)
{
	const int laserpos = 85;
	float min = tlaser[laserpos].dist;
	for(int i=laserpos-2; i<laserpos+2;i++)
	{
		if (tlaser[i].dist < min)
			min = tlaser[i].dist;
	}
	return min;
}

////////////////////////////////////////////////7777
//////////////////////////////////////////////////////

void SpecificWorker::setPick(const Pick& myPick)
{ 
   qDebug() << myPick.x <<":"<< myPick.z ;
   target.copy(myPick.x,myPick.z);
   target.setActive(true); 
   estado=State::INIT;
}

///////////////7

bool SpecificWorker::atTarget()
{

}
void SpecificWorker::go(const string& nodo, const float x, const float y, const float alpha)
{

}
void SpecificWorker::turn(const float speed)
{

}
void SpecificWorker::stop()
{
	
	
}