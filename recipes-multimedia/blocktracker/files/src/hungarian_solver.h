#ifndef HUNGARIAN_SOLVER_H
#define HUNGARIAN_SOLVER_H

#include <vector>

#define N 55 //max number of vertices in one part

class CHungarianSolver {
public:

   typedef std::vector<std::vector<double> > TCostMatrix;

public:
   CHungarianSolver() {}

   void Reset();

   double operator()(const TCostMatrix& t_cost_matrix);

   std::vector<std::pair<int, int> > GetAssignments();

private:
   void Augment(const TCostMatrix& t_cost_matrix);
   
   void InitLabels(const TCostMatrix& t_cost_matrix);

   void UpdateLabels();

   void AddToTree(const TCostMatrix& t_cost_matrix, int n_x, int n_prevx);

private:
   int n, max_match; //n workers and n jobs
   double lx[N], ly[N]; //labels of X and Y parts
   int xy[N]; //xy[x] - vertex that is matched with x,
   int yx[N]; //yx[y] - vertex that is matched with y
   bool S[N], T[N]; //sets S and T in algorithm
   double slack[N]; //as in the algorithm description
   int slackx[N]; //slackx[y] such a vertex, that
   // l(slackx[y]) + l(y) - w(slackx[y],y) = slack[y]
   int prev[N]; //array for memorizing alternating paths
};

#endif







