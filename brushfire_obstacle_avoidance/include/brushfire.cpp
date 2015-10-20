#include <iostream>
#include <vector>
#include <cmath>
#include "float.h"

using namespace std;



// STRUCTURES: -----------------------------------------------------------------

typedef struct Position
{
    int x;
    int y;
    int z;
}Position;



// OTHER FUNCTIONS: -----------------------------------------------------------



float CostFunction(vector< vector<int> >& matrix ,Position position, 
		   int attNum, vector< Position > object, vector<float> factor, vector <float (*)(float,Position,Position)>attfuncList,
		   float repFactor,float (*rep)(float,float)){
    float x = (float)matrix.at(position.x+matrix.size()/2).at(position.y+matrix.front().size()/2);
     x = x/100;
     float value = 0;
    if(x >0.1){
      
      for (int i = 0; i < attNum; i++) {
	  value = value + attfuncList.at(i)(factor.at(i),object.at(i),position);
      }
      
   
      value = value + rep(repFactor,x);
    }
    else{
      return FLT_MAX;
    }
    
    //printf("%f\n",value);
    return value;
}



void displayGrid(vector< vector< int > >& matrix)
{
    int X = matrix.size();
    int Y = matrix.front().size();
    
    for (int i = 0; i < X; ++i)
    {
        for (int j = 0; j < Y; ++j)
        {
            if (matrix[i][j] == 0) cout << "0 ";
            else cout << "\t" << matrix[i][j] << "\t";
        }
        cout << endl;
    }
    
    cout << endl;
}


Position brushfire(vector< vector<int> >& matrix,vector< Position > obstacleList)
{
    int i = 0;
    vector< Position > wavefrontList;
    vector< Position > wavefrontListNext;
    vector< Position > skeleton;
    int halfX = matrix.size()/2;
    int halY = matrix.front().size()/2;
    //printf("halfX:%d,halfY:%d\n",halfX,halY);
    for(vector< vector< int > >::iterator it = matrix.begin(); it !=  matrix.end();it++)
    {
        if ( i != 0 && i != matrix.size()-1 ) {
            int j = 0;
            for(vector<int>::iterator its = it->begin(); its !=  it->end(); its++)
            {
                
                Position x;
                x.x = i-halfX;
                x.y = 0-halY;
                obstacleList.push_back(x);
                Position y;
                y.x = i-halfX;
                y.y = (int)it->size()-1-halY;
                obstacleList.push_back(y);
                j++;
            }
        }
        else{
            int j = 0;
            for(vector<int>::iterator its = it->begin(); its !=  it->end(); its++)
            {
                
                Position x;
                x.x = i-halfX;
                x.y = j-halY;
                obstacleList.push_back(x);
                j++;
            }
        }
        
        i++;
        
    }
    
    //displayGrid(matrix);
    for(vector<Position>::iterator it = obstacleList.begin(); it !=  obstacleList.end(); it++)
    {
        wavefrontList.push_back(*it);
        matrix[it->x+halfX][it->y+halY] = 0;
    }

    //displayGrid(matrix);
    int d = 0;
    bool complete = false;
    while (complete != true) {
        for(vector<Position>::iterator it = wavefrontList.begin(); it !=  wavefrontList.end(); it++)
        {
            int orientation = 0;
            
            if ((it->x+1+halfX < matrix.size() )&& (matrix[it->x+1+halfX][it->y+halY] == -1)) {
                matrix[it->x+1+halfX][it->y+halY] = d+1;
                Position x;
                x.x = it->x+1;
                x.y = it->y;
                wavefrontListNext.push_back(x);
            }
            else{
                orientation++;
            }
            
            if ((it->x-1+halfX < matrix.size() )&&matrix[it->x-1+halfX][it->y+halY] == -1) {
                matrix[it->x-1+halfX][it->y+halY] = d+1;
                Position x;
                x.x = it->x-1;
                x.y = it->y;
                wavefrontListNext.push_back(x);
            }
            else{
                orientation++;
            }
            
            if ((it->y+1+halY < matrix[it->x+halfX].size() )&&matrix[it->x+halfX][it->y+1+halY] == -1) {
                matrix[it->x+halfX][it->y+1+halY] = d+1;
                Position x;
                x.x = it->x;
                x.y = it->y+1;
                wavefrontListNext.push_back(x);
            }
            else{
                orientation++;
            }
            
            if ((it->y-1+halY < matrix[it->x+halfX].size() )&&matrix[it->x+halfX][it->y-1+halY] == -1) {
                matrix[it->x+halfX][it->y-1+halY] = d+1;
                Position x;
                x.x = it->x;
                x.y = it->y-1;
                wavefrontListNext.push_back(x);
            }
            else{
                orientation++;
            }
            
            if (orientation == 4) {
                Position x;
                x.x = it->x;
                x.y = it->y;
                skeleton.push_back(x);
            }
            
        }
        wavefrontList.swap(wavefrontListNext);
        wavefrontListNext.clear();
        d++;
        if (wavefrontList.empty() == true) {
            complete = true;
	    
        }
    }
    
    return skeleton.back();
}
