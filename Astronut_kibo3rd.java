package jp.jaxa.iss.kibo.rpc.sampleapk;


import android.util.Log;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import android.os.SystemClock;


import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.CvType;


import java.util.ArrayList;




public class YourService extends KiboRpcService {
    private final String TAG = this.getClass().getSimpleName();

    @Override
    protected void runPlan1() {
        Quaternion face_airlock = new Quaternion(0f, 0f, -0.707f, 0.707f);
        long Dis_x = 0;
        long Dis_y = 0;
        Mat output = new Mat();

        double r1 = 0.1711585522;
        double[] laser_point = {0.1302, 0.1111};
        double[] target_point = new double[2];


        double[] pos = {11.11, -10, 5.38};


        //start
        Log.i(TAG, "start mission");
        api.startMission();

        //point1
        moveToWrapper(10.71000f, -7.782021f, 4.48000f, 0f, 0.707f, 0f, 0.707f);
        Log.i(TAG, "Arrive A point");
        api.reportPoint1Arrival();
        api.laserControl(true);
        api.takeTarget1Snapshot();
        api.laserControl(false);

        //point2
        moveToWrapper(10.71000f, -7.782021f, 4.87f, 0f, 0f, 0.707f, -0.707f);
        moveToWrapper(10.71000f, -9.89800f, 5.2000f, 0f, 0f, 0.707f, -0.707f);
        //front of target
        moveToWrapper(pos, face_airlock);

        SystemClock.sleep(1500);

        //camera&img processing








        //AR
        double[] target_pos = calTargetPos();


        //cal pivot_to_target
        target_point[0] = target_pos[0] - 0.0572;
        target_point[1] = target_pos[1] + 0.1111;
        double pivot_to_target_length = Math.sqrt(Math.pow(target_point[0], 2) + Math.pow(target_point[1], 2));
        double pivot_laser_target_angle = Math.acos((1+Math.pow(r1,2)-Math.pow(pivot_to_target_length, 2))/2*r1);
        double a = 1;
        double b = 2 * r1 * Math.cos(Math.toRadians( pivot_laser_target_angle));
        double c1 = Math.pow(r1, 2) - Math.pow(pivot_to_target_length, 2);
        double r2 = (-b + Math.sqrt(Math.pow(b, 2) - 4 * a * c1)) / 2 * a;
        //laser coordinate
        double[] laser_shooting_coord = find_laser_point(target_point, r1, r2);
        double laser_origin_to_shooting_length = Math.sqrt(Math.pow(laser_point[0] - laser_shooting_coord[0], 2) + Math.pow(laser_point[1] - laser_shooting_coord[1], 2));
        double pitch = 2 * Math.toDegrees(Math.asin((0.5 * laser_origin_to_shooting_length) / r1));
        //move%rotate

        Quaternion q = eulerToQuaternion(-90, 0, 0);
        pos[2]=pos[2]-target_pos[1];
        moveToWrapper(pos, q);
        pos[0]=pos[0]+target_pos[0];
        Log.i("pos:", String.valueOf(pos[0])+String.valueOf(pos[2]));
        Log.i("pitch:", String.valueOf(pitch));

        moveToWrapper(pos, q);







        //do your job
        api.laserControl(true);
        api.takeTarget2Snapshot();
        api.laserControl(false);
        moveToWrapper(10.71000f, -9.89800f, 5.2000f, 0f, 0f, 0.707f, -0.707f);
        moveToWrapper(10.71000f, -7.782021f, 4.87000f, 0f, 0f, 0.707f, -0.707f);


        //complete
        moveToWrapper(11.27460f, -7.89178f, 4.96538f, 0f, 0f, -0.707f, 0.707f); //goFinal
        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2() {
        // write here your plan 2
    }

    @Override
    protected void runPlan3() {
        // write here your plan 3
    }

    // You can add your method
    private boolean moveToWrapper(double pos_x, double pos_y, double pos_z,
                                  double qua_x, double qua_y, double qua_z,
                                  double qua_w) {

        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        Result result = api.moveTo(point, quaternion, true);
        int loopCounter = 0;
        while (!result.hasSucceeded() && loopCounter < 3) {
            result = api.moveTo(point, quaternion, true);
            loopCounter++;
        }
        Log.d("move[count]", "" + loopCounter);
        return true;

    }


    private Quaternion eulerToQuaternion(double yaw_degree, double pitch_degree, double roll_degree) {
        double yaw = Math.toRadians(yaw_degree); //radian = degree*PI/180
        double pitch = Math.toRadians(pitch_degree);
        double roll = Math.toRadians(roll_degree);

        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);
        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);

        double qx = sr * cp * cy - cr * sp * sy;
        double qy = cr * sp * cy + sr * cp * sy;
        double qz = cr * cp * sy - sr * sp * cy;
        double qw = cr * cp * cy + sr * sp * sy;

        return new Quaternion((float) qx, (float) qy, (float) qz, (float) qw);
    }

    private boolean moveToWrapper(double[] pos, Quaternion quaternion) {

        final Point point = new Point(pos[0], pos[1], pos[2]);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while (!result.hasSucceeded() && loopCounter < 3) {
            result = api.moveTo(point, quaternion, true);
            loopCounter++;
        }
        Log.d("move[count]", "" + loopCounter);
        return true;
    }

    private double[] find_laser_point(double[] target_point, double r1, double r2) {
        double x1 = 0, y1 = 0, x2 = target_point[0], y2 = target_point[1];
        double centerdx = x1 - x2;
        double centerdy = y1 - y2;
        double R = Math.sqrt(centerdx * centerdx + centerdy * centerdy);
        double R2 = R * R;
        double R4 = R2 * R2;
        double a = (r1 * r1 - r2 * r2) / (2 * R2);
        double r2r2 = (r1 * r1 - r2 * r2);
        double c = Math.sqrt(2 * (r1 * r1 + r2 * r2) / R2 - (r2r2 * r2r2) / R4 - 1);
        double fx = (x1 + x2) / 2 + a * (x2 - x1);
        double gx = c * (y2 - y1) / 2;
        double ix1 = fx + gx;
        double ix2 = fx - gx;
        double fy = (y1 + y2) / 2 + a * (y2 - y1);
        double gy = c * (x1 - x2) / 2;
        double iy1 = fy + gy;
        double iy2 = fy - gy;
        if (iy1 > iy2) {
            return new double[]{ix1, iy1};
        }
        return new double[]{ix2, iy2};
    }

    private double[] calTargetPos() {

        double delta_x=0;
        double delta_y=0;
        double[][] cameraParam = api.getNavCamIntrinsics();
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat dstMatrix = new Mat(1, 5, CvType.CV_32FC1);
        cameraMatrix.put(0, 0, cameraParam[0]);
        dstMatrix.put(0, 0, cameraParam[1]);
        Mat output = new Mat();
        Mat ids = new Mat();
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        ArrayList<Mat> corners = new ArrayList<>();
        double xdiff = 0, ydiff = 0;
        double xdiff_k = 0, ydiff_k = 0;
        Mat image2 = api.getMatNavCam();
        api.saveMatImage(image2, "bullseyen.png");
        String default_file = "sdcard/data/jp.jaxa.iss.kibo.rpc.sampleapk/immediate/DebugImages/bullseyen.png";
        Mat src = Imgcodecs.imread(default_file, Imgcodecs.IMREAD_COLOR);

        cameraMatrix.put(0, 0, cameraParam[0]);
        dstMatrix.put(0, 0, cameraParam[1]);
        Imgproc.undistort(src, output, cameraMatrix, dstMatrix);
        Mat gray = new Mat();
        Imgproc.cvtColor(output, gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.medianBlur(gray, gray, 5);
        Mat circles2 = new Mat();
        Imgproc.HoughCircles(gray, circles2, Imgproc.HOUGH_GRADIENT, 5.0,
                (double) gray.rows() / 16, 100.0, 30.0, 20, 35);


        double[] c2 = circles2.get(0, 0);


        long Des_x = Math.round(c2[0]);
        long Des_y = Math.round(c2[1]);
        Log.i("Target_pos:", Des_x + "," + Des_y);
        Mat image3 = output;
        api.saveMatImage(image3, "output.png");




                Mat source = api.getMatNavCam();
                Aruco.detectMarkers(source, dictionary, corners, ids);
                for (int i = 0; i < corners.size(); i++) {
                    //Get shifted aruco tag corners
                    Mat corrected_corner = corners.get(i);
                    //Shift it by the position that it get cropped.
                    corrected_corner.put(0, 0, corrected_corner.get(0, 0)[0] , corrected_corner.get(0, 0)[1] );
                    corrected_corner.put(0, 1, corrected_corner.get(0, 1)[0] , corrected_corner.get(0, 1)[1] );
                    corrected_corner.put(0, 2, corrected_corner.get(0, 2)[0] , corrected_corner.get(0, 2)[1] );
                    corrected_corner.put(0, 3, corrected_corner.get(0, 3)[0] , corrected_corner.get(0, 3)[1] );
                    //tmp mat to store undistorted corners.
                    Mat tmp = new Mat(1, 4, CvType.CV_32FC2);
                    //undistort the corners.
                    Imgproc.undistortPoints(corners.get(i), tmp, cameraMatrix, dstMatrix, new Mat(), cameraMatrix);
                    //put it back in to the same array list.
                    corners.set(i, tmp);
                }

        if (ids.size().height == 4) {
            float markerSize = 0.05f;
            double avg_ar_size = 0;
            double tx_undistort = 0, ty_undistort = 0;

            for (Mat corner : corners) {
                double _x = 0;
                double _y = 0;
                for (int j = 0; j < corner.size().width; j++) {
                    _x = _x + corner.get(0, j)[0];
                    _y = _y + corner.get(0, j)[1];
                }
                avg_ar_size += Math.abs(corner.get(0, 0)[0] - corner.get(0, 1)[0]);
                avg_ar_size += Math.abs(corner.get(0, 2)[0] - corner.get(0, 3)[0]);
                avg_ar_size += Math.abs(corner.get(0, 0)[1] - corner.get(0, 3)[1]);
                avg_ar_size += Math.abs(corner.get(0, 1)[1] - corner.get(0, 2)[1]);
                tx_undistort += _x / 4.0;
                ty_undistort += _y / 4.0;
            }
            tx_undistort /= 4;
            ty_undistort /= 4;
            avg_ar_size /= 16;
            double pixelPerM = avg_ar_size / markerSize;
            xdiff_k = (tx_undistort - 640) / pixelPerM;
            ydiff_k = (480 - ty_undistort) / pixelPerM;
            Log.i("AR[pixelperM]", String.valueOf(pixelPerM) );
            if((xdiff_k+11.11)!=11.2143522418){
                delta_x=(xdiff_k+11.11)-11.2143522418;
            }else{
                delta_x=0;
            }
            if((5.38-ydiff_k)!=5.48534660841031108){
                delta_y=5.48534660841031108-(5.38-ydiff_k);
            }else{
                delta_y=0;
            }

            xdiff=(Des_x-640)/pixelPerM;
            ydiff=(480-Des_y)/pixelPerM;
            Log.i("Target_diff", xdiff+"," +ydiff);
            Log.i("Target_diff", xdiff+"," +(5.38-ydiff));
            Log.i("delta", delta_x+"," +delta_y);
            xdiff=((Des_x-640)/pixelPerM)-delta_x;
            ydiff=((480-Des_y)/pixelPerM)-delta_y;
            Log.i("Target_diff", xdiff+"," +ydiff);
            Log.i("Target_diff", xdiff+"," +(5.38-ydiff));
            Log.i("Target_diff_mid", xdiff_k+"," +ydiff_k);
            Log.i("error_mid", String.valueOf((xdiff_k - 0.1094)+11.11)+","+String.valueOf(5.38-ydiff_k));


        }



        return new double[]{xdiff, ydiff};
    }
}
