#include <assert.h>
#include <math.h>
#include <FL/Fl.H>
#include <FL/Fl_Image.H>
#include "features.h"
#include "ImageLib/FileIO.h"

#define PI 3.14159265358979323846

// Compute features of an image.
bool computeFeatures(CFloatImage &image, FeatureSet &features,int featureType) {
	// TODO: Instead of calling dummyComputeFeatures, write your own
	// feature computation routines and call them here.
	switch (featureType) {
	case 1:
        {
            printf("\ndummyComputeFeatures");
            dummyComputeFeatures(image, features);
            break;
        }
	case 2:
        {
            printf("\nComputeHarriesFeatures");
            ComputeHarrisFeatures(image, features);
            break;
        }
	default:
		return false;
	}
    /*
    switch (descriptor) {
        case 1:
        {
            ComputeNaiveFeatures(image,features);
            break;
        }
        default:
            return false;
    }
     */
	// This is just to make sure the IDs are assigned in order, because
	// the ID gets used to index into the feature array.
    //ComputeGradFeatures(image,features);
    ComputeMyFeatures(image,features);
    //ComputeNaiveFeatures(image,features);
	//for (int i=0; i<features.size(); i++) {
	//	features[i].id = i+1;
	//}
    cout<<"finish feature computation"<<endl;
	return true;
}

// Perform a query on the database.  This simply runs matchFeatures on
// each image in the database, and returns the feature set of the best
// matching image.
bool performQuery(const FeatureSet &f, const ImageDatabase &db, int &bestIndex, vector<FeatureMatch> &bestMatches, double &bestScore, int matchType) {
	// Here's a nice low number.
	bestScore = -1e100;
    printf("\nPerform Query");
	vector<FeatureMatch> tempMatches;
	double tempScore;

	for (unsigned int i=0; i<db.size(); i++) {
		if (!matchFeatures(f, db[i].features, tempMatches, tempScore, matchType)) {
			return false;
		}

		if (tempScore > bestScore) {
			bestIndex = i;
			bestScore = tempScore;
			bestMatches = tempMatches;
		}
	}

	return true;
}

// Match one feature set with another.
bool matchFeatures(const FeatureSet &f1, const FeatureSet &f2, vector<FeatureMatch> &matches, double &totalScore, int matchType) {
	// TODO: We have given you the ssd matching function, you must write your own
	// feature matching function for the ratio test.
	
	printf("\nMatching features.......\n");

	switch (matchType) {
	case 1:
		ssdMatchFeatures(f1, f2, matches, totalScore);
		return true;
	case 2:
		ratioMatchFeatures(f1, f2, matches, totalScore);
		return true;
    case 3:
        meanratioMatchFeatures(f1,f2,matches,totalScore);
        return true;
	default:
		return false;
	}
}

// Evaluate a match using a ground truth homography.  This computes the
// average SSD distance between the matched feature points and
// the actual transformed positions.
double evaluateMatch(const FeatureSet &f1, const FeatureSet &f2, const vector<FeatureMatch> &matches, double h[9]) {
	double d = 0;
	int n = 0;

	double xNew;
	double yNew;
    printf("\n evaluateMatch");

    unsigned int num_matches = matches.size();
	for (unsigned int i=0; i<num_matches; i++) {
		int id1 = matches[i].id1;
        int id2 = matches[i].id2;
        applyHomography(f1[id1-1].x, f1[id1-1].y, xNew, yNew, h);
		d += sqrt(pow(xNew-f2[id2-1].x,2)+pow(yNew-f2[id2-1].y,2));
		n++;
	}	

	return d / n;
}

void addRocData(const FeatureSet &f1, const FeatureSet &f2, const vector<FeatureMatch> &matches, double h[9],vector<bool> &isMatch,double threshold,double &maxD) {
	double d = 0;

	double xNew;
	double yNew;

    unsigned int num_matches = matches.size();
	for (unsigned int i=0; i<num_matches; i++) {
		int id1 = matches[i].id1;
        int id2 = matches[i].id2;
		applyHomography(f1[id1-1].x, f1[id1-1].y, xNew, yNew, h);

		// Ignore unmatched points.  There might be a better way to
		// handle this.
		d = sqrt(pow(xNew-f2[id2-1].x,2)+pow(yNew-f2[id2-1].y,2));
		if (d<=threshold)
		{
			isMatch.push_back(1);
		}
		else
		{
			isMatch.push_back(0);
		}

		if (matches[i].score>maxD)
			maxD=matches[i].score;
	}	
}

vector<ROCPoint> computeRocCurve(vector<FeatureMatch> &matches,vector<bool> &isMatch,vector<double> &thresholds)
{
	vector<ROCPoint> dataPoints;

	for (int i=0; i < (int)thresholds.size();i++)
	{
		//printf("Checking threshold: %lf.\r\n",thresholds[i]);
		int tp=0;
		int actualCorrect=0;
		int fp=0;
		int actualError=0;
		int total=0;

        int num_matches = (int) matches.size();
		for (int j=0;j < num_matches;j++)
		{
			if (isMatch[j])
			{
				actualCorrect++;
				if (matches[j].score<thresholds[i])
				{
					tp++;
				}
			}
			else
			{
				actualError++;
				if (matches[j].score<thresholds[i])
				{
					fp++;
				}
            }
			
			total++;
		}

		ROCPoint newPoint;
		//printf("newPoints: %lf,%lf",newPoint.trueRate,newPoint.falseRate);
		newPoint.trueRate=(double(tp)/actualCorrect);
		newPoint.falseRate=(double(fp)/actualError);
		//printf("newPoints: %lf,%lf",newPoint.trueRate,newPoint.falseRate);

		dataPoints.push_back(newPoint);
	}

	return dataPoints;
}


// Compute silly example features.  This doesn't do anything
// meaningful.
void dummyComputeFeatures(CFloatImage &image, FeatureSet &features) {
	CShape sh = image.Shape();
	Feature f;

	for (int y=0; y<sh.height; y++) {
		for (int x=0; x<sh.width; x++) {
			double r = image.Pixel(x,y,0);
			double g = image.Pixel(x,y,1);
			double b = image.Pixel(x,y,2);

			if (r == 1) {
				// If the pixel satisfies this meaningless criterion,
				// make it a feature.
				
				f.type = 1;
				f.id += 1;
				f.x = x;
				f.y = y;

				f.data.resize(1);
				f.data[0] = r + g + b;

				features.push_back(f);
			}
		}
	}
}

void ComputeHarrisFeatures(CFloatImage &image, FeatureSet &features)
{
	//Create grayscale image used for Harris detection
	CFloatImage grayImage=ConvertToGray(image);

	//Create image to store Harris values
	CFloatImage harrisImage(image.Shape().width,image.Shape().height,1);

	//Create image to store local maximum harris values as 1, other pixels 0
	CByteImage harrisMaxImage(image.Shape().width,image.Shape().height,1);

	
	//compute Harris values puts harris values at each pixel position in harrisImage. 
	//You'll need to implement this function.
    computeHarrisValues(grayImage, harrisImage);
    cout<<"begin ANMS"<<endl;
    //omputeANMS(harrisImage,harrisMaxImage);
	// Threshold the harris image and compute local maxima.  You'll need to implement this function.
	computeLocalMaxima(harrisImage,harrisMaxImage);
    
    // Prints out the harris image for debugging purposes
	CByteImage tmp(harrisImage.Shape());
	convertToByteImage(harrisImage, tmp);
    WriteFile(tmp, "harris.tga");
    

	// TO DO--------------------------------------------------------------------
	//Loop through feature points in harrisMaxImage and create feature descriptor 
	//for each point above a threshold
    int id = 1;
    for (int y=0;y<harrisMaxImage.Shape().height;y++) {
		for (int x=0;x<harrisMaxImage.Shape().width;x++) {
		
			// Skip over non-maxima
            if (harrisMaxImage.Pixel(x, y, 0) == 0)
                continue;

            //TO DO---------------------------------------------------------------------
		    // Fill in feature with descriptor data here. 
            Feature f;
            f.id = id;
            f.x = x;
            f.y = y;
            f.angleRadians = 0;
            id ++;

            // Add the feature to the list of features
            features.push_back(f);
        }
	}
}


//TO DO---------------------------------------------------------------------
//Loop through the image to compute the harris corner values as described in class
// srcImage:  grayscale of original image
// harrisImage:  populate the harris values per pixel in this image
void computeHarrisValues(CFloatImage &srcImage, CFloatImage &harrisImage)
{
    
	int w = srcImage.Shape().width;
    int h = srcImage.Shape().height;
    
    CFloatImage dstx(harrisImage.Shape());
    CFloatImage dsty(harrisImage.Shape());
    //Using sobel kernel smoothing and get convolution/derivatives.
    Convolve(srcImage,dstx,ConvolveKernel_SobelX);
    Convolve(srcImage,dsty,ConvolveKernel_SobelY);
    
    CFloatImage I_xx(harrisImage.Shape());
    CFloatImage I_yy(harrisImage.Shape());
    CFloatImage I_xy(harrisImage.Shape());
    

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            I_xx.Pixel(x,y,0) = dstx.Pixel(x,y,0)*dstx.Pixel(x,y,0);
            I_yy.Pixel(x,y,0) = dsty.Pixel(x,y,0)*dsty.Pixel(x,y,0);
            I_xy.Pixel(x,y,0) = dstx.Pixel(x,y,0)*dsty.Pixel(x,y,0);
        }
    }
    CFloatImage gaussian_weight(CShape(5,5,1));
    for (int i = 0; i < 25; i++)
        gaussian_weight.Pixel(int(i/5),i%5,0) = gaussian5x5[i];
    Convolve(I_xx,I_xx,gaussian_weight);
    Convolve(I_yy,I_yy,gaussian_weight);
    Convolve(I_xy,I_xy,gaussian_weight);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            // TODO:  Compute the harris score for 'srcImage' at this pixel and store in 'harrisImage'.  See the project
            //   c(H) = determinant(H) / trace(H);
            /*harrisImage.Pixel(x,y,0) = (I_xx.Pixel(x,y,0)*I_yy.Pixel(x,y,0)-I_xy.Pixel(x,y,0)*I_xy.Pixel(x,y,0))/(I_xx.Pixel(x,y,0) + I_yy.Pixel(x,y,0));*/
            harrisImage.Pixel(x,y,0) = (I_xx.Pixel(x,y,0)*I_yy.Pixel(x,y,0)-I_xy.Pixel(x,y,0)*I_xy.Pixel(x,y,0))/(I_xx.Pixel(x,y,0) + I_yy.Pixel(x,y,0));
        }
    }
    
}
class ANMS_point{
    public:
    ANMS_point(int x, int y, double distance):p_x(x),p_y(y),dist(distance){}
    int p_x;
    int p_y;
    double dist;
    bool operator > (const ANMS_point &m)const{
        return dist > m.dist;
    }
};
//srcImage: image with Harris values
//destImage: Assign 1 to a pixel if it is above a threshold and is the local maximum in 3x3 window, 0 otherwise.
//    You'll need to find a good threshold to use.
void computeANMS(CFloatImage &srcImage, CByteImage &destImage)
{
    int w = srcImage.Shape().width;
    int h = srcImage.Shape().height;
    vector<ANMS_point> vect;
    double maxi_temp = 0.0;
    const double c_robust = 0.9;
    const int num_features = 500;
    int max_x = 0;
    int max_y = 0;
    for (int y = 0; y < h; y++){
        for (int x = 0; x < w; x++){
            if (srcImage.Pixel(x,y,0) >= 0.1){
                destImage.Pixel(x,y,0) = 1;
            }else{
                destImage.Pixel(x,y,0) = 0;
            }
        }
    }
    for (int y = 0; y < h; y++){
        for (int x = 0; x < w; x++){
            if (destImage.Pixel(x,y,0) != 0){
                double cur_value = srcImage.Pixel(x,y,0);
                for (int j = -1; j <=1; j++)
                    for ( int i = -1; i<=1; i++){
                        if (i == 0 && j == 0)
                            continue;
                        if (x+i >= 0 && x+i < w && y+j >= 0 && y+j < h)
                            if(srcImage.Pixel(x+i,y+j,0)<cur_value)
                                destImage.Pixel(x+i,y+j,0) = 0;
                    }
            }
        }
    }
    for (int y=0; y<h; y++){
        for(int x=0; x<w;x++){
            ANMS_point temp(x,y,0.0);
            vect.push_back(temp);
            if(isnan(srcImage.Pixel(x,y,0))==false && destImage.Pixel(x,y,0) == 1){
                if(srcImage.Pixel(x,y,0) > maxi_temp){
                    maxi_temp = srcImage.Pixel(x,y,0);
                    max_x = x;
                    max_y = y;
                }
            }
        }
    }
    cout<<"finish initialization"<<endl;
    vect[max_y*w+max_x].dist = maxi_temp;
    for (int y=0; y<h; y++){
        for(int x=0;x<w;x++){
            if(isnan(srcImage.Pixel(x,y,0))==false && destImage.Pixel(x,y,0) == 1){
                if (srcImage.Pixel(x,y,0) > c_robust*maxi_temp)
                    vect[y*w+x].dist = 99999.9;
                else
                {
                    double temp_min_distance = 99999.9;
                    for (int m = 0;m<h;m++){
                        for (int n=0;n<w;n++){
                            if(isnan(srcImage.Pixel(m,n,0))==false && destImage.Pixel(m,n,0) == 1){
                                if(srcImage.Pixel(x,y,0) <  c_robust*srcImage.Pixel(m,n,0)){
                                    double cur_distance = double(sqrt((m-x)*(m-x)+(n-y)*(n-y)));
                                    if (cur_distance < temp_min_distance)
                                        temp_min_distance = cur_distance;
                                }
                            }
                        }
                    }
                    vect[y*w+x].dist = temp_min_distance;
                }
            }
        }
    }
    cout<<"finish ANMS"<<endl;
    partial_sort(vect.begin(),vect.begin()+200,vect.end(),greater<ANMS_point>());
    for (int y = 0; y < h; y++){
        for (int x = 0; x < w; x++){
            destImage.Pixel(x,y,0) = 0;
        }
    }
    for (int i=0;i<200;i++){
        destImage.Pixel(vect[i].p_x,vect[i].p_y,0) = 1;
        cout<<"destImage.dist: "<<vect[i].dist<<"x"<<vect[i].p_x<<"y"<<vect[i].p_y<<endl;
        cout<<"harrisvalue:"<<srcImage.Pixel(vect[i].p_x,vect[i].p_y,0)<<endl;
    }
}

// TO DO---------------------------------------------------------------------
// Loop through the harrisImage to threshold and compute the local maxima in a neighborhood
// srcImage:  image with Harris values
// destImage: Assign 1 to a pixel if it is above a threshold and is the local maximum in 3x3 window, 0 otherwise.
//    You'll need to find a good threshold to use.
void computeLocalMaxima(CFloatImage &srcImage,CByteImage &destImage)
{
    int w = srcImage.Shape().width;
    int h = srcImage.Shape().height;
    //0.3 with score 248.79, auc 0.91;0.35 with score 162.857, auc 0.881919; 0.25 with score 376.923, auc 0.920005;
    //0.2 with score ,auc ;
    vector<ANMS_point> vect;
    double average_num = 0;
    double local_maximal_count = 0;
    double local_maximal_sum = 0;
    for (int y = 0; y < h; y++){
        for (int x = 0; x < w; x++){
            if (isnan(srcImage.Pixel(x,y,0)) == false){
                average_num += srcImage.Pixel(x,y,0);
                double cur_value = srcImage.Pixel(x,y,0);
                int temp = 0,total_temp = 0;
                for (int j = -1; j <=1; j++)
                    for ( int i = -1; i<=1; i++){
                        if (i == 0 && j == 0)
                            continue;
                        if (x+i >= 0 && x+i < w && y+j >= 0 && y+j < h)
                            total_temp += 1;
                            if(srcImage.Pixel(x+i,y+j,0)<cur_value)
                                temp += 1;
                    }
                if (temp == total_temp){
                    local_maximal_sum += cur_value;
                    local_maximal_count += 1;
                }
            }
            
        }
    }
    
    cout<<endl;
    cout<<"h:"<<h<<"w:"<<w<<endl;
    cout<<"Harris value sum is: "<<average_num<<endl;
    average_num = average_num / (h*w);
    cout<<"Harris average value is:"<<average_num<<endl;
    cout<<"Local maximal number is:"<<local_maximal_count<<endl;
    cout<<"Harris local maximal average is:"<<double(local_maximal_sum/local_maximal_count)<<endl;
    
    double threshold = 0.1 +double(local_maximal_sum/local_maximal_count);
    cout<<"Threshold is:"<<threshold<<endl;
    for (int y = 0; y < h; y++){
        for (int x = 0; x < w; x++){
            if (srcImage.Pixel(x,y,0) >= threshold){
                destImage.Pixel(x,y,0) = 1;
            }else{
                destImage.Pixel(x,y,0) = 0;
            }
        }
    }
    for (int y = 0; y < h; y++){
        for (int x = 0; x < w; x++){
            if (destImage.Pixel(x,y,0) != 0){
                double cur_value = srcImage.Pixel(x,y,0);
                for (int j = -1; j <=1; j++)
                    for ( int i = -1; i<=1; i++){
                        if (i == 0 && j == 0)
                            continue;
                        if (x+i >= 0 && x+i < w && y+j >= 0 && y+j < h)
                            if(srcImage.Pixel(x+i,y+j,0)<cur_value)
                                destImage.Pixel(x+i,y+j,0) = 0;
                    }
            }
        }
    }
    /*
    for (int y = 0; y < h; y++){
        for (int x = 0; x < w; x++){
            if(destImage.Pixel(x,y,0) = 1){
                vect.push_back(ANMS_point(x,y,srcImage.Pixel(x,y,0)));
            }
        }
    }
    
    partial_sort(vect.begin(),vect.begin()+500,vect.end(),greater<ANMS_point>());
    for (int y = 0; y < h; y++){
        for (int x = 0; x < w; x++){
            destImage.Pixel(x,y,0) = 0;
        }
    }
    for (int i=0;i<500;i++){
        destImage.Pixel(vect[i].p_x,vect[i].p_y,0) = 1;
    }
     */
	
}
void ComputeGradFeatures(CFloatImage &image, FeatureSet & features){
    cout<<"Compute Grad features"<<endl;
    CFloatImage grayImage = ConvertToGray(image);
    int w = image.Shape().width;
    int h = image.Shape().height;
    CFloatImage dstx(grayImage.Shape());
    CFloatImage dsty(grayImage.Shape());
    //Using sobel kernel and get convolution/derivatives.
    //ConvolveKernel_simple.
    Convolve(grayImage,dstx,ConvolveKernel_SobelX);
    Convolve(grayImage,dsty,ConvolveKernel_SobelY);
    vector<Feature>::iterator i = features.begin();
    while (i != features.end()) {
        Feature &f = *i;
        int w = image.Shape().width;
        int h = image.Shape().height;
        for(int j=-2;j<=2;j++){
            for (int k=-2;k<=2;k++){
                if (f.x+k >= 0 && f.x+k <w && f.y+j>=0 && f.y+j<h){
                    double magnitude = sqrt(dsty.Pixel(f.x + k,f.y + j,0)*dsty.Pixel(f.x + k,f.y + j,0)+dstx.Pixel(f.x + k,f.y + j,0)*dstx.Pixel(f.x + k,f.y + j,0));
                    f.data.push_back(magnitude);
                }else{
                    f.data.push_back(0);
                }
                
            }
        }
        i++;
    }
}
void ComputeMyFeatures(CFloatImage &image, FeatureSet & features){
    cout<<"Compute my features"<<endl;
    CFloatImage grayImage = ConvertToGray(image);
    int w = image.Shape().width;
    int h = image.Shape().height;
    const long double pi = 3.141592653589793238L;
    CFloatImage dstx(grayImage.Shape());
    CFloatImage dsty(grayImage.Shape());
    //Using sobel kernel and get convolution/derivatives.
    //ConvolveKernel_simple.
    Convolve(grayImage,dstx,ConvolveKernel_SobelX);
    Convolve(grayImage,dsty,ConvolveKernel_SobelY);
    vector<Feature>::iterator i = features.begin();
    while (i != features.end()) {
        double bin[19];
        for (int n=0;n<19;n++){
            bin[n] = 0;
        }
        Feature &f = *i;
        int w = image.Shape().width;
        int h = image.Shape().height;
        int scale_num = 0;
        // choosing points as 11x11 square
        for (int j = -5; j <= 5; j++){
            for (int k = -5; k <= 5; k++){
                if (f.x+k >= 0 && f.x+k <w && f.y+j>=0 && f.y+j<h){
                    double angle = atan(double(dsty.Pixel(f.x + k,f.y + j,0)/dstx.Pixel(f.x + k,f.y + j,0)));
                    if (isnan(angle)==true)
                        if (dsty.Pixel(f.x+k,f.y+j,0) >= 0)
                            angle = pi/2.0;
                        else
                            angle = -pi/2.0;
                    int index_1 = int((angle+(pi/2.0))/(pi/9.0));
                    if (index_1 > 8){
                        cout<<"warning";
                        break;
                    }
                    double index_2 = double((angle+(pi/2.0))/(pi/9.0))-index_1;
                    double magnitude = sqrt(dsty.Pixel(f.x + k,f.y + j,0)*dsty.Pixel(f.x + k,f.y + j,0)+dstx.Pixel(f.x + k,f.y + j,0)*dstx.Pixel(f.x + k,f.y + j,0));
                    //cout<<"dstx: "<<dstx.Pixel(f.x + k,f.y + j,0)<<"dsty: "<<dsty.Pixel(f.x + k,f.y + j,0)<<"angle: "<<angle<<"index1: "<<index_1<<endl<<"index2: "<<index_2<<"magnitude: "<<magnitude<<endl;
                    if (dsty.Pixel(f.x + k,f.y + j,0) > 0 && dstx.Pixel(f.x + k,f.y + j,0) > 0){
                        bin[index_1+1] += index_2*magnitude;
                        bin[index_1] += (1.0-index_2)*magnitude;
                    }
                    if (dsty.Pixel(f.x + k,f.y + j,0) < 0 && dstx.Pixel(f.x + k,f.y + j,0) < 0){
                        bin[index_1+1+9] += index_2*magnitude;
                        bin[index_1+9] += (1.0-index_2)*magnitude;
                    }
                    if (dsty.Pixel(f.x + k,f.y + j,0) < 0 && dstx.Pixel(f.x + k,f.y + j,0) > 0){
                        bin[index_1+1] += index_2*magnitude;
                        bin[index_1] += (1.0-index_2)*magnitude;
                    }
                    if (dsty.Pixel(f.x + k,f.y + j,0) > 0 && dstx.Pixel(f.x + k,f.y + j,0) < 0){
                        bin[index_1+1+9] += index_2*magnitude;
                        bin[index_1+9] += (1.0-index_2)*magnitude;
                    }
                    scale_num += 1;
                    //f.data.push_back(angle);
                } else {
                    //f.data.push_back(0);
                }
            }
        }
        for (int m = 0;m<19;m++){
            f.data.push_back(double(bin[m]/scale_num));
            //f.data.push_back(bin[m]);
            //cout<<"bin:"<<m<<" value is: "<<bin[m]<<endl;
        }
        i++;
    }
        
    cout<<"Finish"<<endl;
    
}
//using 5x5 windows to describe features in a naive way.
void ComputeNaiveFeatures(CFloatImage &image, FeatureSet &features){
    CFloatImage grayImage = ConvertToGray(image);
    vector<Feature>::iterator i = features.begin();
    while (i != features.end()) {
        Feature &f = *i;
        int w = image.Shape().width;
        int h = image.Shape().height;
        
        // choosing points as 5x5 square
        for (int j = -2; j <= 2; j++){
            for (int k = -2; k <= 2; k++){
                if (f.x+k >= 0 && f.x+k <w && f.y+j>=0 && f.y+j<h){
                    f.data.push_back(grayImage.Pixel(f.x + k,f.y+j,1));
                } else {
                    f.data.push_back(0);
                }
            }
        }
        i++;
    }
    
}
// Perform simple feature matching.  This just uses the SSD
// distance between two feature vectors, and matches a feature in the
// first image with the closest feature in the second image.  It can
// match multiple features in the first image to the same feature in
// the second image.
void ssdMatchFeatures(const FeatureSet &f1, const FeatureSet &f2, vector<FeatureMatch> &matches, double &totalScore) {
	int m = f1.size();
	int n = f2.size();

	matches.resize(m);
	totalScore = 0;

	double d;
	double dBest;
	int idBest;

	for (int i=0; i<m; i++) {
		dBest = 1e100;
		idBest = 0;

		for (int j=0; j<n; j++) {
			d = distanceSSD(f1[i].data, f2[j].data);

			if (d < dBest) {
				dBest = d;
				idBest = f2[j].id;
			}
		}

        matches[i].id1 = f1[i].id;
		matches[i].id2 = idBest;
		matches[i].score = dBest;
		totalScore += matches[i].score;
	}
    printf("score:%f\n",totalScore);
}

// TODO: Write this function to perform ratio feature matching.  
// This just uses the ratio of the SSD distance of the two best matches as the score
// and matches a feature in the first image with the closest feature in the second image.
// It can match multiple features in the first image to the same feature in
// the second image.  (See class notes for more information, and the sshMatchFeatures function above as a reference)
void ratioMatchFeatures(const FeatureSet &f1, const FeatureSet &f2, vector<FeatureMatch> &matches, double &totalScore) 
{
    int m = f1.size();
    int n = f2.size();
    matches.resize(m);
    totalScore = 0;
    double d;
    double dBest;
    double dSecond;
    int idBest;
    for (int i =0; i<m;i++){
        dBest = 1e100;
        dSecond = dBest;
        idBest = 0;
        for (int j=0;j<n;j++){
            d = distanceSSD(f1[i].data,f2[j].data);
            if (d < dBest){
                dSecond = dBest;
                dBest = d;
                idBest = f2[j].id;
            }
        }
        matches[i].id1 = f1[i].id;
        matches[i].id2 = idBest;
        if (dSecond == 0.0)
            matches[i].score = 0;
        else
            matches[i].score = double(dBest/dSecond);
        totalScore += matches[i].score;
    }
    printf("score:%f\n",totalScore);
    
}
void meanratioMatchFeatures(const FeatureSet &f1,const FeatureSet &f2, vector<FeatureMatch> &matches, double &totalScore){
    int m = f1.size();
    int n = f2.size();
    matches.resize(m);
    totalScore = 0;
    double d;
    double dBest;
    double daverage;
    int idBest;
    for (int i =0; i<m;i++){
        dBest = 1e100;
        daverage = 0;
        idBest = 0;
        for (int j=0;j<n;j++){
            d = distanceSSD(f1[i].data,f2[j].data);
            daverage += d;
            if (d < dBest){
                dBest = d;
                idBest = f2[j].id;
            }
        }
        matches[i].id1 = f1[i].id;
        matches[i].id2 = idBest;
        matches[i].score = double(dBest*n/daverage);
        totalScore += matches[i].score;
    }
    printf("score:%f\n",totalScore);
    
}

// Convert Fl_Image to CFloatImage.
bool convertImage(const Fl_Image *image, CFloatImage &convertedImage) {
	if (image == NULL) {
		return false;
	}

	// Let's not handle indexed color images.
	if (image->count() != 1) {
		return false;
	}

	int w = image->w();
	int h = image->h();
	int d = image->d();

	// Get the image data.
	const char *const *data = image->data();

	int index = 0;

	for (int y=0; y<h; y++) {
		for (int x=0; x<w; x++) {
			if (d < 3) {
				// If there are fewer than 3 channels, just use the
				// first one for all colors.
				convertedImage.Pixel(x,y,0) = ((uchar) data[0][index]) / 255.0f;
				convertedImage.Pixel(x,y,1) = ((uchar) data[0][index]) / 255.0f;
				convertedImage.Pixel(x,y,2) = ((uchar) data[0][index]) / 255.0f;
			}
			else {
				// Otherwise, use the first 3.
				convertedImage.Pixel(x,y,0) = ((uchar) data[0][index]) / 255.0f;
				convertedImage.Pixel(x,y,1) = ((uchar) data[0][index+1]) / 255.0f;
				convertedImage.Pixel(x,y,2) = ((uchar) data[0][index+2]) / 255.0f;
			}

			index += d;
		}
	}
	
	return true;
}

// Convert CFloatImage to CByteImage.
void convertToByteImage(CFloatImage &floatImage, CByteImage &byteImage) {
	CShape sh = floatImage.Shape();

    assert(floatImage.Shape().nBands == byteImage.Shape().nBands);
	for (int y=0; y<sh.height; y++) {
		for (int x=0; x<sh.width; x++) {
			for (int c=0; c<sh.nBands; c++) {
				float value = floor(255*floatImage.Pixel(x,y,c) + 0.5f);

				if (value < byteImage.MinVal()) {
					value = byteImage.MinVal();
				}
				else if (value > byteImage.MaxVal()) {
					value = byteImage.MaxVal();
				}

				// We have to flip the image and reverse the color
				// channels to get it to come out right.  How silly!
				byteImage.Pixel(x,sh.height-y-1,sh.nBands-c-1) = (uchar) value;
			}
		}
	}
}

// Compute SSD distance between two vectors.
double distanceSSD(const vector<double> &v1, const vector<double> &v2) {
	int m = v1.size();
	int n = v2.size();

	if (m != n) {
		// Here's a big number.
		return 1e100;
	}

	double dist = 0;

	for (int i=0; i<m; i++) {
		dist += pow(v1[i]-v2[i], 2);
	}

	
	return sqrt(dist);
}

// Transform point by homography.
void applyHomography(double x, double y, double &xNew, double &yNew, double h[9]) {
	double d = h[6]*x + h[7]*y + h[8];

	xNew = (h[0]*x + h[1]*y + h[2]) / d;
	yNew = (h[3]*x + h[4]*y + h[5]) / d;
}

// Compute AUC given a ROC curve
double computeAUC(vector<ROCPoint> &results)
{
	double auc=0;
	double xdiff,ydiff;
	for (int i = 1; i < (int) results.size(); i++)
    {
        //fprintf(stream,"%lf\t%lf\t%lf\n",thresholdList[i],results[i].falseRate,results[i].trueRate);
		xdiff=(results[i].falseRate-results[i-1].falseRate);
		ydiff=(results[i].trueRate-results[i-1].trueRate);
		auc=auc+xdiff*results[i-1].trueRate+xdiff*ydiff/2;
    	    
    }
	return auc;
}