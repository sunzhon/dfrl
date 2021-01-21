/*
 * ExtensorReflex.cpp
 *
 *  Created on: 2018-3-9
 *      Author: suntao
 */

#include "ExtensorReflex.h"

namespace stcontroller {

ExtensorReflex::ExtensorReflex(int _leg) {
	// 0) parameters
	legId=_leg;
	// 1) new the ANNs
	vrn = new VRN();
	setDefaultTransferFunction(identityFunction());
	setNeuronNumber(4);
	addSubnet(vrn);
	//2) connect the neural network
		// oriNP network
	w(vrn->getNeuron(0), ANN::getNeuron(0),1.57);
	w(vrn->getNeuron(1), ANN::getNeuron(1),5.0);
		
	// outN connect to itself
	w(getNeuron(2), vrn->getNeuron(6), 0.5);
	w(getNeuron(3), vrn->getNeuron(6), -1.0);

}
ExtensorReflex::~ExtensorReflex() {
	// TODO Auto-generated destructor stub
	delete vrn;
}
void ExtensorReflex::setInput(double _leg1, double _leg2,double _leg3,double _leg4, double _grf) {
	leg1 = _leg1;
	leg2 = _leg2;
	leg3 = _leg3;
	leg4 = _leg4;
	grf = _grf;
}
void ExtensorReflex::step() {
	double inputValue;
	switch(legId){
	case 0:
		inputValue = leg2+leg3 +leg4;
		break;
	case 1:
		inputValue = leg1+leg3 +leg4;
		break;
	case 2:
		inputValue = leg1+leg2 +leg4;
		break;
	default:
		inputValue = leg1+leg2 +leg3;
	}
	//3) set the inputs of the ANN
	ANN::setInput(0,inputValue);
	ANN::setInput(1,0.2*grf);
	if(legId==1)
		ANN::setInput(1,0.2);
	//4) update
	updateActivities();
	updateOutputs();
	
}

double ExtensorReflex::getOutput(unsigned int index) {
	assert(index < getNeuronNumber());
		return ANN::getOutput(index+2);
}



/*      
      
RegressionDataset ExtensorReflex::loadData( const string& dataFile, const string& labelFile )  
    {  
        Data<RealVector> inputs;  
        Data<RealVector> label;  
        try  
        {  
            importCSV( inputs, dataFile, ' ' );  
            importCSV( label, labelFile, ' ' );  
        }  
        catch ( ... )  
        {  
            cerr << "Unable to open file " <<  dataFile << " and/or " << labelFile << ". Check paths!" << endl;  
            exit( EXIT_FAILURE );  
        }  
        RegressionDataset data( inputs, label );  
        return data;  
    }  
      
void ExtensorReflex::testML(int argc, char **argv)  
    {  
   if(argc < 2) {
		cerr << "usage: " << argv[0] << " (filename)" << endl;
		exit(EXIT_FAILURE);
	}
	//###begin<load_data>
	ClassificationDataset data;
	try {
		importCSV(data, argv[1], LAST_COLUMN, ' ');
	} 
	catch (...) {
		cerr << "unable to read data from file " <<  argv[1] << endl;
		exit(EXIT_FAILURE);
	}
	//###end<load_data>
	
	//create a test and training partition of the data
	//###begin<split_data>
	ClassificationDataset test = splitAtElement(data,static_cast<std::size_t>(0.8*data.numberOfElements()));
	//###end<split_data>
	
	//###begin<objects>
	//create a classifier for the problem
	LinearClassifier<> classifier;
	//create the lda trainer
	LDA lda;
	//###end<objects>
	//train the classifier using the training portion of the Data
	//###begin<train>
	lda.train(classifier,data);
	//###end<train>

	//###begin<eval>
	ZeroOneLoss<> loss;
	double error = loss(test.labels(),classifier(test.inputs()));
	//###end<eval>
	
	//print results
	//###begin<outputs>
	cout << "RESULTS: " << endl;
	cout << "========\n" << endl;
	cout << "test data size: " << test.numberOfElements() << endl;
	cout << "error rate: " << error << endl;
   }
*/
} /* namespace stcontroller */
