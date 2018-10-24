#include <visp/vpFeaturePoint.h>
#include <ecn_baxter_vs/baxter_arm.h>
#include <visp/vpSubMatrix.h>
#include <visp/vpSubColVector.h>
#include <ecn_common/vpQuadProg.h>
#include <ecn_common/visp_utils.h>
//#include <ecn_sensorbased/optim.h>

using namespace std;

/**
 * Function to calculate h-weight for joints
 * @param center Middle of safe interval
 * @param qmin Obvious
 * @param qmax Obvious
 * @param q Current joint state
 * @param margin Margin to calculate point, where weight should be activated.
 * @return Value of h-weight.
 */
double CalcH(double center, double qmin, double qmax, double q, double margin){
    double q_act, h;
    if (q < center){ // min
        q_act = qmin + margin * (qmax - qmin);
        h = ecn::weight(-q, -q_act, -qmin);
    } else{ // max
        q_act = qmax - margin * (qmax - qmin);
        h = ecn::weight(q, q_act, qmax);
    }
    return h;
}

int main(int argc, char** argv)
{
    // update your group name to work on the real Baxter
    std::string group_name = "SORRY";

    BaxterArm arm(argc, argv, group_name);    // defaults to simulation with right arm
    //BaxterArm arm(argc, argv, group_name, false, "left");   // real robot with left arm


    // Baxter initial joint states
    vpColVector q = arm.init();
    // Baxter joint limits
    vpColVector qmin = arm.jointMin(),
                qmax = arm.jointMax();

    // Baxter desired joint states for task 2 of he lab
    vpColVector q_star = (qmax + qmin)/2;
//    vpColVector q_star = arm.init(); // last 4 joints for 3rd task of the lab
//    q_star[0] = (qmax[0] + qmin[0])/2;
//    q_star[1] = (qmax[1] + qmin[1])/2;
//    q_star[2] = (qmax[2] + qmin[2])/2;

    // define a simple 2D point feature and its desired value
    vpFeaturePoint p, pd;
    pd.set_xyZ(0,0,1);

    // the error
    vpColVector e(3);
    double x, y, area;
    // desired area
    double area_d = arm.area_d();
    // lambda
    double lambda = 0.7;
    // margin for weight activation, from interval (0; 0.5)
    double margin = 0.1;

    // loop variables
    vpColVector qdot;
    // simple control
    vpMatrix Lxy(2, 6), L(3, 6), Jq(6, 7), Js(3,7);
    // additional matrices for control
    vpMatrix H(10, 10), Jh(10, 7), I7(7, 7), HJh(10, 7);
    H.eye();
    I7.eye();
    // total error and joint error vectors
    vpColVector e_tot(10), e_q(7);

    while(arm.ok())
        {
            cout << "-------------" << endl;

            // get point features
            x = arm.x();
            y = arm.y();
            area = arm.area();
            p.set_xyZ(x, y, 1);
            std::cout << "x: " << x << ", y: 1" << y << ", area: " << area << '\n';

            // update error vector e
            e[0] = x;
            e[1] = y;
            e[2] = area - area_d;

            ecn::putAt(e_tot, e, 0);

            // joint errors
            q = arm.jointPosition();
            ecn::putAt(e_tot, (q - q_star), 3);

            // update interaction matrix L
            Lxy = p.interaction(vpBasicFeature::FEATURE_ALL);
            ecn::putAt(L, Lxy, 0, 0); // Common 2D point interaction matrix
            // Line for area in intereaction matrix
            L[2][2] = 2 * area;
            L[2][3] = 3 * area * y;
            L[2][4] = -3 * area * x;

            // compute feature Jacobians from L and cameraJacobian
            Jq = arm.cameraJacobian();
            Js = L * Jq;

            ecn::putAt(Jh, Js, 0, 0);
            ecn::putAt(Jh, I7, 3, 0);

            // build H matrix (2nd section)
            H[3][3] = CalcH(q_star[0], qmin[0], qmax[0], q[0], margin);
            H[4][4] = CalcH(q_star[1], qmin[1], qmax[1], q[1], margin);
            H[5][5] = CalcH(q_star[2], qmin[2], qmax[2], q[2], margin);
            // comment following h's for the third part of the lab
            H[6][6] = CalcH(q_star[3], qmin[3], qmax[3], q[3], margin);
            H[7][7] = CalcH(q_star[4], qmin[4], qmax[4], q[4], margin);
            H[8][8] = CalcH(q_star[5], qmin[5], qmax[5], q[5], margin);
            H[9][9] = CalcH(q_star[6], qmin[6], qmax[6], q[6], margin);

            // compute control
            HJh = H * Jh;
            qdot = -lambda * HJh.pseudoInverse() * H * e_tot;

            // compute simple control input w/out joint limits for task 1
//            qdot = -lambda * Js.pseudoInverse() * e;

            // send this command to the robot
            arm.setJointVelocity(qdot);

            // display current joint positions and VS error
            arm.plot(e);
    }
}
