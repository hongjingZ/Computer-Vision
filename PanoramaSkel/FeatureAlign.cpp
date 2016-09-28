///////////////////////////////////////////////////////////////////////////
//
// NAME
//  FeatureAlign.h -- image registration using feature matching
//
// SEE ALSO
//  FeatureAlign.h      longer description
//
// Copyright ?Richard Szeliski, 2001.  See Copyright.h for more details
// (modified for CSE576 Spring 2005)
//
///////////////////////////////////////////////////////////////////////////

#include "ImageLib/ImageLib.h"
#include "FeatureAlign.h"
#include <math.h>
#include <iostream>

/******************* TO DO *********************
* alignPair:
*	INPUT:
*		f1, f2: source feature sets
*		matches: correspondences between f1 and f2
*               *NOTE* Each match in 'matches' contains two feature ids of matching features, id1 (in f1) and id2 (in f2).
*               These ids are 1-based indices into the feature arrays,
*               so you access the appropriate features as f1[id1-1] and f2[id2-1].
*		m: motion model
*		f: focal length
*		nRANSAC: number of RANSAC iterations
*		RANSACthresh: RANSAC distance threshold
*		M: transformation matrix (output)
*	OUTPUT:
*		repeat for nRANSAC iterations:
*			choose a minimal set of feature matches
*			estimate the transformation implied by these matches
*			count the number of inliers
*		for the transformation with the maximum number of inliers,
*		compute the least squares motion estimate using the inliers,
*		and store it in M
*/
int alignPair(const FeatureSet &f1, const FeatureSet &f2,
              const vector<FeatureMatch> &matches, MotionModel m, float f,
              int nRANSAC, double RANSACthresh, CTransform3x3& M)
{
    // BEGIN TODO
    // write this entire method
    vector< vector<int> > total_inliers;
    int maxIndex = 0;
    int maxSize = 0;
    for (int i=0; i<nRANSAC;i++)
    {
        int Index1 = 0;
        int Index2 = 0;
        do {
            Index1 = rand() % matches.size();
            Index2 = rand() % matches.size();
        } while (Index1 == Index2);
        vector<int> inliers_list;
        inliers_list.push_back(Index1);
        inliers_list.push_back(Index2);
        CTransform3x3 tempMatrix;
        leastSquaresFit(f1,f2,matches,m,f,inliers_list,tempMatrix);
        //cout<<"tempMatrix[0]"<<tempMatrix[0][0]<<tempMatrix[0][1]<<tempMatrix[0][2]<<endl;
        //cout<<"tempMatrix[1]"<<tempMatrix[1][0]<<tempMatrix[1][1]<<tempMatrix[1][2]<<endl;
        //cout<<"tempMatrix[2]"<<tempMatrix[2][0]<<tempMatrix[2][1]<<tempMatrix[2][2]<<endl;
        countInliers(f1,f2,matches,m,f,tempMatrix,RANSACthresh,inliers_list);
        //cout<<"After counting:"<<inliers_list.size()<<endl;
        //cout<<"tempMatrix[0]"<<tempMatrix[0][0]<<tempMatrix[0][1]<<tempMatrix[0][2]<<endl;
        //cout<<"tempMatrix[1]"<<tempMatrix[1][0]<<tempMatrix[1][1]<<tempMatrix[1][2]<<endl;
        //cout<<"tempMatrix[2]"<<tempMatrix[2][0]<<tempMatrix[2][1]<<tempMatrix[2][2]<<endl;
        total_inliers.push_back(inliers_list);
        if (inliers_list.size() > maxSize)
        {
            maxSize = inliers_list.size();
            maxIndex = i;
        }
    }
    cout<<"num_inliers: "<<maxSize<<endl;
    leastSquaresFit(f1,f2,matches,m,f,total_inliers[maxIndex],M);
    // END TODO
    cout<<"U: "<<M[0][2]<<endl;
    cout<<"V: "<<M[1][2]<<endl;
    return 0;
}

/******************* TO DO *********************
* countInliers:
*	INPUT:
*		f1, f2: source feature sets
*		matches: correspondences between f1 and f2
*               *NOTE* Each match contains two feature ids of matching features, id1 (in f1) and id2 (in f2).
*               These ids are 1-based indices into the feature arrays,
*               so you access the appropriate features as f1[id1-1] and f2[id2-1].
*		m: motion model
*		f: focal length
*		M: transformation matrix
*		RANSACthresh: RANSAC distance threshold
*		inliers: inlier feature IDs
*	OUTPUT:
*		transform the matched features in f1 by M
*
*		count the number of matching features for which the transformed
*		feature f1[id1-1] is within SSD distance RANSACthresh of its match
*		f2[id2-1]
*
*		store the indices of these matches in inliers
*
*		
*/
int countInliers(const FeatureSet &f1, const FeatureSet &f2,
                 const vector<FeatureMatch> &matches, MotionModel m, float f,
                 CTransform3x3 M, double RANSACthresh, vector<int> &inliers)
{
    inliers.clear();
    int count = 0;
    
    for (unsigned int i=0; i<(int) matches.size(); i++) {
        // BEGIN TODO
        // determine if the ith matched feature f1[id1-1], when transformed by M,
        // is within RANSACthresh of its match in f2
        //
        // if so, increment count and append i to inliers
        //
        // *NOTE* Each match contains two feature ids of matching features, id1 and id2.
        //        These ids are 1-based indices into the feature arrays,
        //        so you access the appropriate features as f1[id1-1] and f2[id2-1].
        CVector3 p1;
        p1[0] = f1[matches[i].id1-1].x;
        p1[1] = f1[matches[i].id1-1].y;
        p1[2] = 1;
        CVector3 p2;
        p2[0] = f2[matches[i].id2-1].x;
        p2[1] = f2[matches[i].id2-1].y;
        p2[2] = 1;
        p1 = M * p1;
        p1[0] = p1[0]/p1[2];
        p1[1] = p1[1]/p1[2];
        double ssd = pow(p1[0]-p2[0],2)+pow(p1[1]-p2[1],2);
        //cout<<"PU:"<<M[0][2]<<"PV"<<M[1][2]<<endl;
        //cout<<"P1:"<<p1[0]<<","<<p1[1]<<endl;
        //cout<<"P2:"<<p2[0]<<","<<p2[1]<<endl;
        //cout<<"real_pos:"<<f2[matches[i].id2-1].x<<","<<f2[matches[i].id2-1].y<<endl;
        //cout<<"SSD:"<<ssd<<endl;
        if (ssd < pow(RANSACthresh,2))
        {
            inliers.push_back(i);
            count += 1;
            //cout<<"Count"<<count<<endl;
        }
        // END TODO
    }

    return count;
}

/******************* TO DO *********************
* leastSquaresFit:
*	INPUT:
*		f1, f2: source feature sets
*		matches: correspondences between f1 and f2
*		m: motion model
*		f: focal length
*		inliers: inlier match indices (indexes into 'matches' array)
*		M: transformation matrix (output)
*	OUTPUT:
*		compute the transformation from f1 to f2 using only the inliers
*		and return it in M
*/
int leastSquaresFit(const FeatureSet &f1, const FeatureSet &f2,
                    const vector<FeatureMatch> &matches, MotionModel m, float f,
                    const vector<int> &inliers, CTransform3x3& M)
{
    // for project 2, the transformation is a translation and
    // only has two degrees of freedom
    //
    // therefore, we simply compute the average translation vector
    // between the feature in f1 and its match in f2 for all inliers
    double u = 0;
    double v = 0;
    for (int i=0; i<inliers.size(); i++) {
        double xTrans, yTrans;

        // BEGIN TODO
        // compute the translation implied by the ith inlier match
        // and store it in (xTrans,yTrans)
        int M_index = inliers[i];
        xTrans = f2[matches[M_index].id2-1].x - f1[matches[M_index].id1-1].x;
        yTrans = f2[matches[M_index].id2-1].y - f1[matches[M_index].id1-1].y;
        //cout<<"Inlier f1=:"<<"(x,y) = "<<f1[matches[M_index].id1-1].x<<f1[matches[M_index].id1-1].y<<endl;
        //cout<<"Inlier f2=:"<<"(x,y) = "<<f2[matches[M_index].id1-1].x<<f2[matches[M_index].id1-1].y<<endl;
        // END TODO
        u += xTrans;
        v += yTrans;
    }
    u /= inliers.size();
    v /= inliers.size();

    M[0][0] = 1;
    M[0][1] = 0;
    M[0][2] = u;
    M[1][0] = 0;
    M[1][1] = 1;
    M[1][2] = v;
    M[2][0] = 0;
    M[2][1] = 0;
    M[2][2] = 1;
    
    //cout<<"uuu:"<<u<<"vvv:"<<v<<"inliers size:"<<inliers.size()<<endl;
    return 0;
}
