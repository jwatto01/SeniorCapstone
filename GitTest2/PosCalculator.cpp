//position calculator (posCalculator.cpp)
//calculate the position and orientation of a magnet
//
#include "stdafx.h"
#include <iostream>
#include <posCalculator.h>

#define ARDUINO_WAIT_TIME 2000

using namespace alglib;
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3i;
using Eigen::Vector3d;

//const double acceptedError = 1e-12;
//const ae_int_t maxIterations = 100;
const double PI = 3.14159265;
//string portNumber;
//HANDLE hSerial;
//bool connected;
//COMSTAT status;

//void setZeroVals();
//void convertToMicroTesla(const Vector3i &rawData, Vector3d &retArr);
//void meritFunc(const real_1d_array &x, double& fi, void* obj);
//void jacobian(const real_1d_array &x, real_1d_array &fi, real_2d_array &jac, void* obj);
//bool connectArduino(char *portName);
//int readData(char buffer[169]);
//bool writeData(char buffer[1]);
//void updateSensorReadings(char byteBuff[169]);
double dipoleEstimate = 0.3;

//magnet M1;//create magnet with default magnetic dipole moment
//Now create 8 sensors and place them in 3d space with an orientation respective of test-setup (floor)
//note that orientation is in euler angles (phi, theta, psi)!
/*sensor S1({0.161, -0.132, 0.334},{-0.7854, -1.5708, 1.5708}, 1), S2({0.161, 0.132, 0.335},{0.7854, -1.5708, 1.5708}, 2);
sensor S3({0.161, 0.126, 0.086},{-0.7854, -1.5708, 1.5708}, 3), S4({0.161, -0.116, 0.084},{0.7854, -1.5708, 1.5708}, 4);
sensor S5({-0.152, -0.133, 0.3385},{0.7854, -1.5708, -1.5708}, 5), S6({-0.152, 0.132, 0.331},{-0.7854, -1.5708, -1.5708}, 6);
sensor S7({-0.152, 0.129, 0.076},{0.7854, -1.5708, -1.5708}, 7), S8({-0.152,-0.11,0.097},{-0.7854, -1.5708, -1.5708}, 8);*/

//have to validate S5,S7, and S2, S4, S6, S8 with matlab
//sensor S1({0.161, -0.132, 0.334},{-1.5708, 1.5708, 0.7854}, 1), S2({0.161, 0.132, 0.335},{-1.5708, 1.5708, -0.7854}, 2);
//sensor S3({0.161, 0.126, 0.086},{-1.5708, 1.5708, 0.7854}, 3), S4({0.161, -0.116, 0.084},{-1.5708, 1.5708, -0.7854}, 4);
//sensor S5({-0.152, -0.133, 0.3385},{1.5708, 1.5708, -0.7854}, 5), S6({-0.152, 0.132, 0.331},{1.5708, 1.5708, 0.7854}, 6);
//sensor S7({-0.152, 0.129, 0.076},{1.5708, 1.5708, -0.7854}, 7), S8({-0.152,-0.11,0.097},{1.5708, 1.5708, 0.7854}, 8);

//sensor allSensors[8];

Vector3d residual(sensor &curSensor){
	Vector3d expectedMagField = curSensor.calculateMagField(M1);
	Vector3d actualMagField = curSensor.getAvgScaledVal();
	Vector3d difference;
	Vector3d meanNoise = curSensor.getMeanNoise();
	difference = actualMagField - expectedMagField - meanNoise;
	
	return difference;
}

void funcVect(const real_1d_array &x, real_1d_array& fi, void* obj){
	double origMagDipole;
	Matrix3d curSqCoVar, invSQCoVar;
	Vector3d curDiff, magPos, magOr, origMagPos, origMagOr;
	origMagPos = M1.posVal();
	origMagOr = M1.orientation();
	origMagDipole = M1.dipoleMomentVal();
	
	//update magnet parameters
	for(int i = 0; i<3; i++) magPos(i) = x[i];
	for(int i = 0; i<3; i++) magOr(i) = x[i+3];
	M1.updatePosition(magPos);
	M1.updateOrientation(magOr);
	M1.updateDipoleMoment(x[6]);
	
	//calculate fi for all sensors and stack each 3x1 vector from sensor 0 to 7
	for(int i = 0; i<24; i++){
		curSqCoVar = allSensors[(i/3)].getSQRTsampleCoVar();
		invSQCoVar = curSqCoVar.inverse();//find covariance^(-1)

		curDiff = residual(allSensors[(i/3)]);//find ( d-s(x) )
		Vector3d error = invSQCoVar * curDiff;
		//stack the error vector in our function vector, forming a 24x1 vector
		fi[i] = error(0);
		i++;
		fi[i] = error(1);
		i++;
		fi[i] = error(2);
	}

	//restore M1's parameters
	M1.updatePosition(origMagPos);
	M1.updateOrientation(origMagOr);
	M1.updateDipoleMoment(origMagDipole);
}

/*
This algorithm attempts to solve the system of nonlinear equations
    F[0](x[0], ..., x[n-1])   = 0
    F[1](x[0], ..., x[n-1])   = 0
    ...
    F[M-1](x[0], ..., x[n-1]) = 0
*/
//jacobian is partial with respect to functor (24x1 stacked vector of function evaluations for all sensor axis)
void jacobian(const real_1d_array &x, real_1d_array &fi, real_2d_array &jac, void* obj){
	//get curMagnet's position + orientation
	Vector3d origMagPos, origMagOr, curMagPos, curMagOr, curDiff;
	double origMagDipole = M1.dipoleMomentVal();
	Matrix3d invSQCoVar, curSqCoVar;
	for(int i = 0; i<3; i++) curMagPos(i) = x[i];
	for(int i = 0; i<3; i++) curMagOr(i) = x[i+3];
	origMagPos = M1.posVal();
	origMagOr = M1.orientation();
	M1.updatePosition(curMagPos);
	M1.updateOrientation(curMagOr);
	M1.updateDipoleMoment(x[6]);
	double h = 1e-12;
	
	for(int i = 0; i<24; i++){
		curSqCoVar = allSensors[(i/3)].getSQRTsampleCoVar();
		invSQCoVar = curSqCoVar.inverse();//find covariance^(-1)

		curDiff = residual(allSensors[(i/3)]);//find ( d-s(x) )
		Vector3d error = invSQCoVar * curDiff;
		//stack the error vector in our function vector, forming a 25x1 vector
		fi[i] = error(0);
		i++;
		fi[i] = error(1);
		i++;
		fi[i] = error(2);
	}
	
	//partial with respect to function evaluation

	for(int i = 0; i<7; i++){
		//perterb parameters one at a time and calculate partial with respect to function vector, fi
		if(i<3){
			curMagPos(i) += h;
			M1.updatePosition(curMagPos);
		}
		else if(i<6){
			curMagOr(i-3) +=h;
			M1.updateOrientation(curMagOr);
		}
		else{
			M1.updateDipoleMoment(x[6] + h);
		}
		//evaluate funcion for updated position
		for(int j = 0; j<24; j++){
			curSqCoVar = allSensors[(j/3)].getSQRTsampleCoVar();
			invSQCoVar = curSqCoVar.inverse();//find covariance^(-1)
	
			curDiff = residual(allSensors[(j/3)]);//find ( d-s(x) )
			Vector3d error = invSQCoVar * curDiff;
			
			
			jac[j][i] = (error(0)-fi[j])/h;
			j++;
			jac[j][i] = (error(1)-fi[j])/h;
			j++;
			jac[j][i] = (error(2)-fi[j])/h;
		}
		
		//reset magnet position for next calculation
		if(i<3){
			curMagPos(i) -= h;
			M1.updatePosition(curMagPos);
		}
		else if(i<6){
			curMagOr(i-3) -=h;
			M1.updateOrientation(curMagOr);
		}
		else{
			M1.updateDipoleMoment(x[6] - h);
		}
	}
	//restore magnet parameters
	M1.updatePosition(origMagPos);
	M1.updateOrientation(origMagOr);
	M1.updateDipoleMoment(origMagDipole);
}

int mainfn()
{
	//Setup sensors and magnet parameters
	//place magnet somewhere in xyz space to initialize
	Vector3d newMagPos(0.0, 0.0, 0.0);//x, y, z coordinates in meters
	Vector3d newMagOr(0.0,0.0,0.0);//magnet orientation in euclidean angles (start with pointing along z axis)
	M1.updatePosition(newMagPos);
	M1.updateOrientation(newMagOr);
		
	allSensors[0] = S1;
	allSensors[1] = S2;
	allSensors[2] = S3;
	allSensors[3] = S4;
	allSensors[4] = S5;
	allSensors[5] = S6;
	allSensors[6] = S7;
	allSensors[7] = S8;

	int testtt = 0;
    //this is where we requested user input (i.e. button press)

	const ae_int_t n = 7;
	const ae_int_t m = 24;
	Vector3d curMagPos = M1.posVal();
	Vector3d curMagOr = M1.orientation();
	double startPoint[7];
	MatrixXd setOfStartPoints(10,7);
	
	setOfStartPoints << -0.1 , 0.0 , 0.05 , 0.0 , 0.0 , 0.0 , M1.dipoleMomentVal()
					 , 0.0 , 0.1 , 0.05 , 0.0 , 0.0 , 0.0 , M1.dipoleMomentVal()
					 , 0.1 , 0.0 , 0.05 , 0.0 , 0.0 , 0.0 , M1.dipoleMomentVal()
					 , 0.0 , -0.1 , 0.05 , 0.0 , 0.0 , 0.0 , M1.dipoleMomentVal()
					 , 0.0 , 0.0 , 0.05 , 0.0 , 0.0 , 0.0 , M1.dipoleMomentVal()
					 , -0.1 , 0.0 , 0.05 , 0.0 , PI , 0.0 , M1.dipoleMomentVal()
					 , 0.0 , 0.1 , 0.05 , 0.0 , PI , 0.0 , M1.dipoleMomentVal()
					 , 0.1 , 0.0 , 0.05 , 0.0 , PI , 0.0 , M1.dipoleMomentVal()
					 , 0.0 , -0.1 , 0.05 , 0.0 , PI , 0.0 , M1.dipoleMomentVal()
					 , 0.0 , 0.0 , 0.05 , 0.0 , PI , 0.0 , M1.dipoleMomentVal();
	
	real_1d_array x = "[0.0,0.0,0.0,0.0,0.0,0.0,0.0]";
	real_1d_array lbound = "[-0.5,-0.5,0.0,-INF,-INF,-INF,0.1]";
	real_1d_array hbound = "[+0.5,+0.5,+0.5,+INF,+INF,+INF,0.8]";
	real_1d_array scale = "[0.1,0.1,0.1,6.2832,3.1416,6.2832,0.1]";
	minlmstate state;
	minlmreport rep;
	minlmcreatevj(n,m,x,state);
	minlmsetbc(state,lbound,hbound);
//	minlmsetscale(state,scale);
	minlmsetcond(state,1e-14,1e-14,1e-14,1000);
	//start communication with arduino on com port	
	string str = "COM9";
	char * writable = new char[str.size() + 1];
    std::copy(str.begin(), str.end(), writable);
    writable[str.size()] = '\0';
    
	connectArduino(writable);
	delete []writable;
	char dataINBuffer[169];
	char startC[1] = {'S'}, calibChar[1] = {'C'};
	//need to calibrate sensors first

    bool calibrated = false;
	bool foundStdDev = false;
	bool firstMeas = false;
    int numCountTmp = 0;

	while(1){
		//first, signal to arduino that we want new data
		if(calibrated){
			Sleep(10);
			if(writeData(startC)){
				int nCharsRead = 0;
				Sleep(50);
				nCharsRead = readData(dataINBuffer);//Get new packet
				if(nCharsRead == 169)
					updateSensorReadings(dataINBuffer);
				else//error occurred, so lets resignal arduino for data
				{
					cout << "error, read " << nCharsRead << " bytes read from Arduino!" << endl;
					char throwaway;
                    //need to determine how to handle errors (popup forum?)
					cin >> throwaway;//suspend until user tells to continue
					continue;
				}
				if(firstMeas){
					firstMeas = false;
					cout << "Must now take sample of data to acquire sensor standard deviation values" << endl;
				}
                else{
						if(allSensors[7].getNumAvgMeasCount() > 100 && !(allSensors[7].getNumMeasCount()%10)){//take 100 samples first to find the standard dev for all axis
							//re-evaluate magnet's position based on new avg data
							//if nMeasCount % 10 = 0, then we have a new updated average!
							//if(!(allSensors[7].getNumMeasCount() % 1)){
								/*if(x[0] > 1.0 || x[1] > 1.0 || x[2] > 1.0 || x[0] < -1.0 || x[1] < -1.0 || x[2] < 0.0)
								{
									cout << "exceeded bounds, please enter new start position for optimization: ";
									double throwaway;
									for(int l = 0; l<3; l++){
										cin >> throwaway;
										startPoint[l] =  throwaway;
									}
									startPoint[3] = 0.0;
									startPoint[4] = 0.0;
									startPoint[5] = 0.0;
									x.setcontent(6,startPoint);
								}*/
								//update start point with magnet's last updated parameter set
								curMagPos = M1.posVal();
								curMagOr = M1.orientation();
								for(int i = 0; i<3; i++) startPoint[i] = curMagPos(i);
								for(int i = 0; i<3; i++) startPoint[i+3] = curMagOr(i);
								startPoint[6] = M1.dipoleMomentVal();
								
								x.setcontent(7,startPoint);
								minlmrestartfrom(state,x);
								//cout << "before solve" << endl;
								alglib::minlmoptimize(state, funcVect, jacobian);
								//cout << "after solve" << endl;
								minlmresults(state,x,rep);
								cout << rep.terminationtype << endl;
								//cout << rep.iterationscount << endl;
								//cout << rep.nfunc << endl;
								for(int k = 0; k<3; k++) newMagPos(k) = x[k];
								for(int k = 0; k<3; k++) newMagOr(k) = x[k+3];
								M1.updatePosition(newMagPos);
								M1.updateOrientation(newMagOr);
								M1.updateDipoleMoment(x[6]);
								cout << x[6] << endl;

                                //used to print resulting solution to log for analysis via matlab, but now we just want to log them locally.
                                //Need to come up with good method of storing magnet parameters (location and orientation) for both streaming/plotting
                                //Also need to come up with way of storing the sample covariance matricies (i.e. in text file) so we can avoid having to
                                //calibrate each time we wish to track.
								
                                //these two lines will open up a stream with a text file
								ofstream fout;
								fout.open("C:\\Users\\andre_000\\Desktop\\test.txt", ofstream::app);//make last argument ofstream::trunc to only keep one value in there at a time

                                //This will loop through each sensor object and print the magnetic measurements (x, y, and z values) to the text file
								for(int j = 0; j<8; j++){
									Vector3d tmpscaled = allSensors[j].getAvgScaledVal();
									for(int i = 0; i<2; i++) fout << tmpscaled(i) << ",";
									fout << tmpscaled(2) << endl;
								}
                                fout.close();//have to close the stream after writing
								
								//print out avg scaled measurements
								
								//sample covariance, measured x,y,z, expected x,y,z, endl (for all 8 sensors)
								//Vector3d tmpscaled, curMagField, meanNoise;
								//Matrix3d curCoVar;
								//for(int j = 0; j<8; j++){
									//curCoVar = allSensors[j].getsampleCoVar();
									//tmpscaled = allSensors[j].getAvgScaledVal();
									//curMagField = allSensors[j].calculateMagField(M1);
									//meanNoise = allSensors[j].getMeanNoise();
								//for(int k = 0; k<5; k++) fout << x[k] << ","; -------------------------------------------------------------------------------------------------
									//for(int k = 0; k<9; k++) fout << curCoVar(k) << ",";//sample covariance for this sensor
									//for(int k = 0; k<3; k++) fout << tmpscaled(k) - meanNoise(k) << ",";//measured mag field at this sensor
									//for(int k = 0; k<3; k++){
									//	fout << curMagField(k);//expected mag field for this sensor
									//	if(k<2) fout << ",";
									//}
								//fout << x[5] << endl;--------------------------------------------------------------------------------------------------------------------------
								//}
								
								//fout.close();/**/
							//}
					
						}	
					}	
				}
			
			Sleep(10);
		}

	}
	
	return 0;
}

//this will acquire 1000 data samples from which the sampleCoVariance will be calculated from automatically for each sensor object
void gatherSampleCovarData(){
    //first ensure that all the sensor classes contain no data in initialDataSample
    for(int i = 0; i<8; i++){
        allSensors[i].reset();
    }
    //then acquire 1000 samples
    for(int i = 0; i<1000; i++){
        Sleep(10);
        if(writeData(startC)){
            int nCharsRead = 0;
            Sleep(50);
            nCharsRead = readData(dataINBuffer);//Get new packet
            if(nCharsRead == 169)
                updateSensorReadings(dataINBuffer);
            else//error occurred, so lets resignal arduino for data
            {
                cout << "error, read " << nCharsRead << " bytes read from Arduino!" << endl;
                char throwaway;
                //need to determine how to handle errors (popup forum?)
                cin >> throwaway;//suspend until user tells to continue
                continue;
            }
        }
    }

    storeNoiseData(); //store initialD
}

void findFirstLocation(){
    //We've already acquired values of sample covariance in each sensor object
    //This is where we begin tracking, so must be sure that the magnet is in the sensing region

    double bestResults = 1000.0;//bad initial value to be updated below
    VectorXd bestParams(7);
    //get best first solution to setup minlmoptimization
    for(int k = 0; k<7; k++) startPoint[k] = setOfStartPoints(0,k);
    x.setcontent(7,startPoint);
    alglib::minlmoptimize(state, funcVect, jacobian);
    for(int i = 0; i<10; i++){
        //update start point
        for(int k = 0; k<7; k++) startPoint[k] = setOfStartPoints(i,k);

        x.setcontent(7,startPoint);
        minlmrestartfrom(state, x);
        alglib::minlmoptimize(state, funcVect, jacobian);
        minlmresults(state,x,rep);
        if(state.f < bestResults){
            bestResults = state.f;
            cout << state.f << endl;
            for(int j = 0; j<7; j++) bestParams(j) = x[j];
        }
    }/**/
    //provide magnet with best parameters to start with
    for(int j = 0; j<3; j++) curMagPos(j) = bestParams(j);
    for(int j = 0; j<3; j++) curMagOr(j) = bestParams(j+3);
    M1.updatePosition(curMagPos);
    M1.updateOrientation(curMagOr);
    M1.updateDipoleMoment(bestParams(6));
}

void calibrateSystem(){
    for(int k = 0; k<20; k++){
        if(writeData(calibChar)){
            Sleep(800);//give sensors plenty of time to take measurements and do averaging
            if(k==18){
                cout << "last measurement, please rotate sensors to operating position" << endl;
                char throwaway;
                cin >> throwaway;
            }
            else if(k==19){
                cout << "done calibrating!" << endl;
                firstMeas = true;
                calibrated = true;
            }
            else{
                cout << "please rotate sensors." << endl;
                Sleep(500);
            }
        }
    }
}

void setZeroVals(){
	for(int j = 0; j<8; j++){
		allSensors[j].updateOffsets();
	}
}

//Unpack buffer then parse it into an array of ints
//then map to double and update each sensor's measurement datamember
void updateSensorReadings(char byteBuff[169]){
	
	for(int i = 0; i<8; i++){
		Vector3d newSenseVal;
		Vector3i retRawData;
		//unpack data
		for(int j = 0; j<3; j++){
			int unpackedRawData;
			string temp = "";
			for(int k = 0; k<7; k++){
				//ignore all non-important data
				if(byteBuff[i*21+j*7+k] != 'N' && byteBuff[i*21+j*7+k] != '!' && byteBuff[i*21+j*7+k] != ',')
					temp += byteBuff[i*21+j*7+k];//append digit to temp string
			}
			//reconstruct int from temp
			stringstream str(temp);
			str >> unpackedRawData;
			
			retRawData(j) = unpackedRawData;
			//cout << retRawData(j) << " ";
		}
		//cout << endl;
		//retRawData(2) = -retRawData(2);//convert to left-handed coordinate system
		
		//convert raw data to micro teslas
		allSensors[i].updateRawData(retRawData);
		convertToMicroTesla(retRawData, newSenseVal);
		//update current sensor's measurement data
		allSensors[i].updateSenseVal(newSenseVal);
	}
}

void convertToMicroTesla(const Vector3i& rawData, Vector3d &retArr){
	for(int i = 0; i<3; i++)
		retArr(i) = ((double)rawData(i)+30000.0)*(2e-3) / 60000.0 - 1e-3;
}

void storeNoiseData(){

}

bool connectArduino(char *portName){

    //Connects to the port.
    hSerial = CreateFileA(portName,
    GENERIC_READ | GENERIC_WRITE,
    0,
    NULL,
    OPEN_EXISTING,
    FILE_ATTRIBUTE_NORMAL,
    NULL);
	
	COMSTAT status;
    if(hSerial==INVALID_HANDLE_VALUE)
    {
        if(GetLastError()==ERROR_FILE_NOT_FOUND){
            cout << "ERROR: Handle was not attached. Reason: " << portName << " not available.\n";
            return false;
        }
        else
        {
            //If port is in use by another program or did not closed previous process.
            cout << "ERROR!!! \n";
            CloseHandle(hSerial);
            return false;
        }
    }
    else
    {

        DCB dcbSerialParams = {0};

        if (!GetCommState(hSerial, &dcbSerialParams))
        {
            cout << "failed to get current serial parameters!" << endl;
            return false;
        }
        else
        {
            // Set Serial Port specifications.
            dcbSerialParams.BaudRate=CBR_115200;
            dcbSerialParams.ByteSize=8;
            dcbSerialParams.StopBits=ONESTOPBIT;
            dcbSerialParams.Parity=NOPARITY;
			dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;//ensures arduino resets properly
			
             if(!SetCommState(hSerial, &dcbSerialParams))
             {
                cout << "ALERT: Could not set Serial Port parameters";
                return false;
             }
             else
             {
                 connected = true;
                 cout << "Connection Successful for :"<< portName << " !!! \n";
                 //purge com buffer of remaining characters
                 PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
                 //Wait 2s as the arduino board will be reseting
                 Sleep(ARDUINO_WAIT_TIME);
				 return true;
             }
        }
    }

}

int readData(char buffer[169]){
    DWORD bytesRead;
    unsigned int bytesToRead = 0;
    DWORD dwCommModemStatus;
    DWORD errors;
    
    if(hSerial != INVALID_HANDLE_VALUE){
    	
		while(!bytesToRead){//loop until we have non-zero bytes to read from COM
			ClearCommError(hSerial, &errors, &status);
			bytesToRead = status.cbInQue;
		}
		if(ReadFile(hSerial, buffer, bytesToRead, &bytesRead, 0)) return (int)bytesRead;
		
	}
	
    //If nothing has been read, or that an error was detected return -1
    cout << "error occurred while reading data from serial port" << endl;
    return -1;
}

bool writeData(char buffer[1]){
    DWORD bytesSend;

    //Try to write the buffer on the Serial port
    if(!WriteFile(hSerial, buffer, (DWORD)1, &bytesSend, 0))
    {
        cerr << "couldn't write to serial port!" << endl;
        return false;
    }
    else
        return true;
}




