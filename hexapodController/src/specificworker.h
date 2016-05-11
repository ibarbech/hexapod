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
#include <osgviewer/osgview.h>
#include <innermodel/innermodelviewer.h>
#include <qt4/QtCore/QTimer>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


public slots:
	void compute();
	void update();
	void updateState(int state);
	void stateuphexapod();
	void updateposleg();

private:
	QTimer clk;
	InnerModel *inner;
	QString base;
	QStringList legs;
	int X, Y, Z, X_pre, Y_pre, Z_pre, modovalue, modoaux,syn;
	float ik_x, ik_y, ik_z, vel, x, y , z;
	QVec legsp[6], lini, lfin, lmed, lrot, lrot1, lrot2;
	bool IK;
	LegControllerPrx proxies[6];
	int l1[3],l2[3];
	RoboCompLegController::StateLeg statelegs[6];
	RoboCompLegController::AnglesLeg angles;
	
	/*---------------------------------------------*/
	void updateStates();
	void statesmachine();
	void uphexapod();
	void fkLegs();
	void ikLegs();
	void ikBody();
	bool caminar3x3();
	bool rotar();
	
	
	QVec bezier3(QVec p0, QVec p1, QVec p2, float t);
	QVec bezier2(QVec p0, QVec p2, float t);
	
	double mapear(double x, double in_min, double in_max, double out_min, double out_max);
	
	OsgView *osgView;
	InnerModelViewer *innerViewer;
};

#endif

