#include "hwa_lidar_base.h"
#include "hwa_set_lidar.h"
#include <iostream>

using namespace hwa_lidar;
using namespace std;

SO3 hwa_lidar::lidar_base::skew(const Triple& v)
{
    SO3 vnx;
    vnx << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return vnx;
}

void hwa_lidar::lidar_base::readChisquare_test()
{
    const double CHI_SQUARED_TABLE[1000] = {
        NAN,0.003932,0.102587,0.351846,0.710723,1.145476,1.635383,2.167350,2.732637,3.325113,3.940299,4.574813,5.226029,5.891864,6.570631,7.260944,7.961646,8.671760,9.390455,10.117013,
        10.850811,11.591305,12.338015,13.090514,13.848425,14.611408,15.379157,16.151396,16.927875,17.708366,18.492661,19.280569,20.071913,20.866534,21.664281,22.465015,23.268609,24.074943,24.883904,25.695390,
        26.509303,27.325551,28.144049,28.964717,29.787477,30.612259,31.438995,32.267622,33.098077,33.930306,34.764252,35.599864,36.437093,37.275893,38.116218,38.958027,39.801278,40.645933,41.491954,42.339308,
        43.187958,44.037874,44.889024,45.741377,46.594905,47.449581,48.305378,49.162270,50.020233,50.879243,51.739278,52.600315,53.462333,54.325312,55.189231,56.054072,56.919817,57.786447,58.653945,59.522294,
        60.391478,61.261482,62.132291,63.003888,63.876261,64.749396,65.623278,66.497895,67.373234,68.249284,69.126030,70.003463,70.881571,71.760343,72.639768,73.519835,74.400535,75.281858,76.163793,77.046332,
        77.929465,78.813184,79.697479,80.582343,81.467767,82.353742,83.240262,84.127317,85.014902,85.903008,86.791628,87.680755,88.570382,89.460503,90.351111,91.242200,92.133763,93.025794,93.918287,94.811237,
        95.704637,96.598482,97.492766,98.387485,99.282632,100.178202,101.074191,101.970593,102.867404,103.764618,104.662231,105.560239,106.458637,107.357420,108.256584,109.156124,110.056038,110.956320,111.856966,112.757973,
        113.659337,114.561053,115.463118,116.365529,117.268281,118.171372,119.074797,119.978553,120.882637,121.787046,122.691775,123.596823,124.502186,125.407860,126.313843,127.220131,128.126722,129.033613,129.940801,130.848283,
        131.756057,132.664118,133.572466,134.481097,135.390009,136.299198,137.208663,138.118401,139.028410,139.938687,140.849230,141.760036,142.671103,143.582429,144.494011,145.405848,146.317937,147.230276,148.142863,149.055696,
        149.968773,150.882091,151.795649,152.709445,153.623476,154.537742,155.452239,156.366967,157.281923,158.197105,159.112512,160.028141,160.943992,161.860062,162.776350,163.692854,164.609572,165.526502,166.443644,167.360995,
        168.278554,169.196320,170.114290,171.032463,171.950839,172.869414,173.788188,174.707160,175.626327,176.545689,177.465244,178.384991,179.304928,180.225055,181.145368,182.065869,182.986554,183.907423,184.828474,185.749707,
        186.671120,187.592712,188.514481,189.436426,190.358547,191.280841,192.203309,193.125948,194.048758,194.971737,195.894884,196.818199,197.741680,198.665326,199.589135,200.513108,201.437243,202.361538,203.285994,204.210608,
        205.135380,206.060309,206.985394,207.910634,208.836028,209.761574,210.687273,211.613123,212.539123,213.465273,214.391571,215.318016,216.244608,217.171346,218.098229,219.025255,219.952425,220.879738,221.807192,222.734786,
        223.662521,224.590394,225.518406,226.446555,227.374841,228.303263,229.231820,230.160512,231.089337,232.018295,232.947385,233.876607,234.805959,235.735442,236.665053,237.594793,238.524661,239.454656,240.384777,241.315025,
        242.245397,243.175894,244.106514,245.037258,245.968124,246.899112,247.830221,248.761451,249.692800,250.624269,251.555856,252.487562,253.419385,254.351325,255.283380,256.215552,257.147839,258.080240,259.012755,259.945383,
        260.878124,261.810977,262.743942,263.677018,264.610204,265.543500,266.476906,267.410421,268.344044,269.277775,270.211613,271.145558,272.079609,273.013766,273.948028,274.882395,275.816866,276.751441,277.686119,278.620900,
        279.555783,280.490768,281.425855,282.361042,283.296330,284.231717,285.167204,286.102791,287.038475,287.974258,288.910138,289.846116,290.782190,291.718361,292.654628,293.590990,294.527447,295.463998,296.400644,297.337384,
        298.274217,299.211143,300.148162,301.085272,302.022475,302.959769,303.897153,304.834628,305.772194,306.709849,307.647593,308.585427,309.523349,310.461359,311.399458,312.337643,313.275916,314.214276,315.152721,316.091253,
        317.029871,317.968574,318.907362,319.846234,320.785191,321.724231,322.663356,323.602563,324.541853,325.481226,326.420681,327.360218,328.299836,329.239536,330.179316,331.119177,332.059118,332.999139,333.939240,334.879420,
        335.819679,336.760016,337.700432,338.640926,339.581498,340.522147,341.462873,342.403677,343.344556,344.285512,345.226544,346.167651,347.108834,348.050092,348.991424,349.932831,350.874312,351.815868,352.757497,353.699199,
        354.640974,355.582822,356.524743,357.466736,358.408801,359.350937,360.293146,361.235425,362.177775,363.120196,364.062688,365.005249,365.947881,366.890582,367.833353,368.776192,369.719101,370.662078,371.605124,372.548238,
        373.491420,374.434669,375.377986,376.321370,377.264821,378.208339,379.151923,380.095574,381.039290,381.983072,382.926920,383.870833,384.814812,385.758855,386.702962,387.647135,388.591371,389.535671,390.480036,391.424463,
        392.368954,393.313508,394.258125,395.202805,396.147547,397.092352,398.037218,398.982146,399.927136,400.872188,401.817300,402.762474,403.707708,404.653003,405.598359,406.543774,407.489250,408.434785,409.380380,410.326035,
        411.271749,412.217521,413.163353,414.109243,415.055192,416.001199,416.947264,417.893387,418.839568,419.785806,420.732101,421.678454,422.624864,423.571330,424.517853,425.464433,426.411068,427.357760,428.304508,429.251312,
        430.198171,431.145085,432.092055,433.039079,433.986159,434.933293,435.880482,436.827725,437.775022,438.722373,439.669779,440.617237,441.564750,442.512316,443.459934,444.407606,445.355331,446.303109,447.250939,448.198822,
        449.146756,450.094743,451.042782,451.990873,452.939015,453.887209,454.835453,455.783750,456.732097,457.680495,458.628944,459.577443,460.525993,461.474593,462.423243,463.371943,464.320693,465.269492,466.218341,467.167240,
        468.116187,469.065184,470.014230,470.963325,471.912468,472.861660,473.810900,474.760189,475.709526,476.658910,477.608343,478.557823,479.507351,480.456926,481.406549,482.356219,483.305936,484.255699,485.205510,486.155367,
        487.105271,488.055221,489.005218,489.955260,490.905349,491.855484,492.805664,493.755890,494.706161,495.656478,496.606840,497.557248,498.507700,499.458197,500.408739,501.359326,502.309957,503.260632,504.211352,505.162116,
        506.112924,507.063776,508.014672,508.965612,509.916595,510.867621,511.818691,512.769805,513.720961,514.672160,515.623402,516.574687,517.526015,518.477385,519.428798,520.380253,521.331750,522.283290,523.234871,524.186494,
        525.138160,526.089866,527.041615,527.993404,528.945236,529.897108,530.849022,531.800976,532.752972,533.705008,534.657085,535.609203,536.561361,537.513560,538.465799,539.418078,540.370398,541.322757,542.275156,543.227596,
        544.180074,545.132593,546.085151,547.037748,547.990385,548.943061,549.895776,550.848530,551.801323,552.754155,553.707025,554.659934,555.612882,556.565868,557.518893,558.471956,559.425057,560.378196,561.331373,562.284588,
        563.237841,564.191131,565.144459,566.097825,567.051228,568.004669,568.958146,569.911661,570.865213,571.818802,572.772428,573.726091,574.679790,575.633526,576.587299,577.541108,578.494953,579.448835,580.402753,581.356707,
        582.310697,583.264723,584.218785,585.172883,586.127016,587.081185,588.035390,588.989630,589.943905,590.898216,591.852562,592.806943,593.761359,594.715810,595.670296,596.624817,597.579373,598.533963,599.488588,600.443247,
        601.397940,602.352668,603.307431,604.262227,605.217058,606.171922,607.126821,608.081753,609.036719,609.991719,610.946752,611.901820,612.856920,613.812054,614.767221,615.722422,616.677656,617.632923,618.588223,619.543556,
        620.498922,621.454320,622.409752,623.365216,624.320712,625.276242,626.231803,627.187398,628.143024,629.098683,630.054374,631.010097,631.965852,632.921639,633.877458,634.833309,635.789191,636.745106,637.701051,638.657029,
        639.613038,640.569078,641.525150,642.481253,643.437388,644.393553,645.349750,646.305977,647.262236,648.218525,649.174846,650.131197,651.087578,652.043991,653.000434,653.956907,654.913411,655.869946,656.826510,657.783105,
        658.739730,659.696386,660.653071,661.609786,662.566531,663.523307,664.480112,665.436946,666.393811,667.350705,668.307628,669.264581,670.221564,671.178576,672.135617,673.092688,674.049788,675.006917,675.964075,676.921262,
        677.878478,678.835723,679.792997,680.750299,681.707631,682.664991,683.622379,684.579796,685.537242,686.494716,687.452219,688.409750,689.367309,690.324897,691.282512,692.240156,693.197828,694.155528,695.113255,696.071011,
        697.028794,697.986606,698.944445,699.902311,700.860206,701.818127,702.776077,703.734054,704.692058,705.650089,706.608148,707.566234,708.524348,709.482488,710.440655,711.398850,712.357071,713.315320,714.273595,715.231897,
        716.190226,717.148582,718.106964,719.065373,720.023809,720.982271,721.940759,722.899274,723.857815,724.816383,725.774976,726.733596,727.692243,728.650915,729.609613,730.568338,731.527088,732.485864,733.444666,734.403494,
        735.362348,736.321228,737.280133,738.239063,739.198020,740.157002,741.116009,742.075042,743.034100,743.993183,744.952292,745.911426,746.870586,747.829770,748.788980,749.748214,750.707474,751.666758,752.626068,753.585402,
        754.544761,755.504145,756.463554,757.422987,758.382446,759.341928,760.301436,761.260967,762.220524,763.180104,764.139709,765.099339,766.058992,767.018670,767.978372,768.938099,769.897849,770.857624,771.817422,772.777245,
        773.737091,774.696962,775.656856,776.616774,777.576716,778.536682,779.496671,780.456684,781.416720,782.376781,783.336864,784.296971,785.257102,786.217256,787.177433,788.137634,789.097858,790.058105,791.018376,791.978669,
        792.938986,793.899326,794.859688,795.820074,796.780483,797.740915,798.701369,799.661847,800.622347,801.582870,802.543415,803.503984,804.464575,805.425188,806.385824,807.346483,808.307164,809.267868,810.228594,811.189342,
        812.150113,813.110906,814.071721,815.032559,815.993418,816.954300,817.915204,818.876130,819.837078,820.798048,821.759040,822.720054,823.681090,824.642147,825.603227,826.564328,827.525451,828.486596,829.447762,830.408950,
        831.370159,832.331391,833.292643,834.253917,835.215213,836.176530,837.137868,838.099228,839.060609,840.022011,840.983434,841.944879,842.906345,843.867832,844.829340,845.790869,846.752419,847.713990,848.675582,849.637196,
        850.598829,851.560484,852.522160,853.483856,854.445573,855.407311,856.369070,857.330849,858.292649,859.254469,860.216310,861.178172,862.140054,863.101956,864.063879,865.025822,865.987786,866.949770,867.911774,868.873798,
        869.835843,870.797908,871.759993,872.722098,873.684223,874.646369,875.608534,876.570720,877.532925,878.495150,879.457395,880.419660,881.381945,882.344250,883.306575,884.268919,885.231283,886.193666,887.156070,888.118493,
        889.080935,890.043397,891.005879,891.968380,892.930901,893.893441,894.856000,895.818579,896.781177,897.743795,898.706432,899.669088,900.631763,901.594458,902.557171,903.519904,904.482656,905.445427,906.408217,907.371027,
        908.333855,909.296702,910.259568,911.222453,912.185357,913.148279,914.111221,915.074181,916.037160,917.000158,917.963175,918.926210,919.889264,920.852336,921.815427,922.778537,923.741665,924.704812,925.667977,926.631161,
    };
    for (int i = 1; i < 1000; i++)
    {
        chi_squared_test_table[i] = CHI_SQUARED_TABLE[i];
    }

}

hwa_lidar::lidar_base::lidar_base(set_base* set):lidar_state_id(0), lidar_next_id(0)
{
    scan_observation_noise =dynamic_cast<set_lidar*>(set)->scan_observation_noise();
    scan_observation_noise *= scan_observation_noise;

    map_observation_noise = dynamic_cast<set_lidar*>(set)->map_observation_noise();
    map_observation_noise *= map_observation_noise;
    
    R_lidar_imu= dynamic_cast<set_lidar*>(set)->R_lidar_imu();
    t_lidar_imu = dynamic_cast<set_lidar*>(set)->t_lidar_imu();
    T_lidar_imu = dynamic_cast<set_lidar*>(set)->T_lidar_imu();
    window_size = dynamic_cast<set_lidar*>(set)->window_size();
    use_scan = dynamic_cast<set_lidar*>(set)->use_scan();
    use_map = dynamic_cast<set_lidar*>(set)->use_map();
    use_pp = dynamic_cast<set_lidar*>(set)->use_pp();
    use_segmenter = dynamic_cast<set_lidar*>(set)->use_segmenter();
    use_corrdistort = dynamic_cast<set_lidar*>(set)->distortion();

    lidarproc = new lidar_frame(set);

    readChisquare_test();
}

hwa_lidar::lidar_base::~lidar_base()
{
    if(lidarproc!=nullptr)
        delete lidarproc;
}



void hwa_lidar::lidar_base::Huber(Matrix &H,Vector &r, float ther)
{
    ///< temporarily useless
    double tmp = ther;
    r = tmp * r;
    H = tmp * H;

    /*if (r.norm() < ther)
    {
        return;
    }
    else if (r.norm() > ther)
    {
        double tmp = ther+2*ther*r.norm() /0.1;
        r = tmp * r;
        H = tmp * H;
    }
    else
    {
        double tmp = sqrt(2 * r.norm()*ther - ther * ther) / r.norm();
        r = tmp*r;
        H = tmp * H;
    }*/
}

void hwa_lidar::lidar_base::lidarMeasurementJacobian(lidarOdometryObs obs, Matrix &H, Vector &r,int id,bool use_3d,float ther)
{
    cout << "lidarodometry" << endl;
    cout << "corner points:" << obs.cornerFeature.currCornerPointCloud.size() << endl;
    cout << "surf points:" << obs.surfFeature.currSurfPointCloud.size() << endl;
    cout << "lidar size:" << lidar_states.size() << endl;

    int corner_obsnum= obs.cornerFeature.currCornerPointCloud.size();
    int surf_obsnum = obs.surfFeature.currSurfPointCloud.size();
    int state_num = 0;
    if (lidarproc->estimate_extrinsic)
    {
        state_num = lidar_states.size() * 6 + 6;
    }
    else
    {
        state_num = lidar_states.size() * 6;
    }

    if (use_3d)
        corner_obsnum = corner_obsnum *3;

    Matrix corner_H = Matrix::Zero(corner_obsnum, state_num);
    Vector corner_r = Vector::Zero(corner_obsnum);
    Matrix surf_H = Matrix::Zero(surf_obsnum, state_num);
    Vector surf_r = Vector::Zero(surf_obsnum);

    for (int i = 0; i < obs.cornerFeature.currCornerPointCloud.size(); i++)
    {
        if (use_3d)
        {
            ///< initialize the H matrix
            Matrix H_dp = Matrix::Zero(3, 3);
            Matrix H_px = Matrix::Zero(3, state_num);
            ///< initialize the r matrix
            Vector rl = Vector::Zero(3);

            ///< calculate the residual of observation(distance between point and line)
            Triple v_lj = obs.cornerFeature.correspondCornerPointCloudA[i] - obs.cornerFeature.correspondCornerPointCloudB[i];
            Triple v_li = obs.cornerFeature.currCornerPointCloud_inlast[i] - obs.cornerFeature.correspondCornerPointCloudB[i];
            Triple v_ji = obs.cornerFeature.currCornerPointCloud_inlast[i] - obs.cornerFeature.correspondCornerPointCloudA[i];
            Triple vjl_i = v_ji.cross(v_li);
            Triple vl_ji = v_lj.cross(v_li);
            double length_lj = sqrt(v_lj(0)*v_lj(0) + v_lj(1)*v_lj(1) + v_lj(2)*v_lj(2));

            H_dp(0, 1) = -v_lj(2) / length_lj;
            H_dp(0, 2) = v_lj(1) / length_lj;
            H_dp(1, 0) = v_lj(2) / length_lj;
            H_dp(1, 2) = -v_lj(0) / length_lj;
            H_dp(2, 0) = -v_lj(1) / length_lj;
            H_dp(2, 1) = v_lj(0) / length_lj;


            rl(0) = vl_ji.x() / length_lj;
            rl(1) = vl_ji.y() / length_lj;    
            rl(2) = vl_ji.z() / length_lj;

            SO3 Rk = obs.curr_R_l_e;
            SO3 Rk_1 = obs.last_R_l_e;
            Triple pk = obs.curr_t_l_e;
            Triple pk_1 = obs.last_t_l_e;

            if (lidarproc->estimate_extrinsic)
            {
                if (!first_odo)
                {
                    H_px.block<3, 3>(0, 6 * id + 6) = skewSymmetric(Rk_1.transpose() * (Rk * obs.cornerFeature.currCornerPointCloud[i] + pk - pk_1));
                    H_px.block<3, 3>(0, 6 * id + 3 + 6) = -Rk_1.transpose();
                }
                H_px.block<3, 3>(0, 6 * id + 6 + 6) = -Rk_1.transpose() * Rk * skewSymmetric(obs.cornerFeature.currCornerPointCloud[i]);
                H_px.block<3, 3>(0, 6 * id + 9 + 6) = Rk_1.transpose();
            }
            else
            {
                if (!first_odo)
                {
                    H_px.block<3, 3>(0, 6 * id) = skewSymmetric(Rk_1.transpose() * (Rk * obs.cornerFeature.currCornerPointCloud[i] + pk - pk_1));
                    H_px.block<3, 3>(0, 6 * id + 3) = -Rk_1.transpose();
                }
                H_px.block<3, 3>(0, 6 * id + 6) = -Rk_1.transpose() * Rk * skewSymmetric(obs.cornerFeature.currCornerPointCloud[i]);
                H_px.block<3, 3>(0, 6 * id + 9) = Rk_1.transpose();
            }

            Matrix temp_H = H_dp * H_px;
            corner_H.row(i * 3) = temp_H.row(0);
            corner_H.row(i * 3 + 1) = temp_H.row(1);
            corner_H.row(i * 3 + 2) = temp_H.row(2);
            corner_r(i * 3) = rl(0);
            corner_r(i * 3 + 1) = rl(1);
            corner_r(i * 3 + 2) = rl(2);
        }
        else
        {
            ///< initialize the H matrix
            Matrix H_dp = Matrix::Zero(1, 3);
            Matrix H_px = Matrix::Zero(3, state_num);
            ///< initialize the r matrix
            Vector rl = Vector::Zero(1);

            ///< calculate the residual of observation(distance between the point and line)
            Triple v_lj = obs.cornerFeature.correspondCornerPointCloudA[i] - obs.cornerFeature.correspondCornerPointCloudB[i];
            Triple v_li = obs.cornerFeature.currCornerPointCloud_inlast[i] - obs.cornerFeature.correspondCornerPointCloudB[i];
            Triple v_ljxi = v_lj.cross(v_li);
            double length_lj = sqrt(v_lj(0)*v_lj(0) + v_lj(1)*v_lj(1) + v_lj(2)*v_lj(2));
            double length_ljxi = sqrt(v_ljxi(0)*v_ljxi(0) + v_ljxi(1)*v_ljxi(1) + v_ljxi(2)*v_ljxi(2));
            double d_e = length_ljxi / length_lj;

            double a = v_lj(1)*v_li(2) - v_lj(2)*v_li(1);
            double b = v_lj(2)*v_li(0) - v_lj(0)*v_li(2);
            double c = v_lj(0)*v_li(1) - v_lj(1)*v_li(0);

            H_dp(0, 0) = -(v_lj(2)*b - v_lj(1)*c) / length_lj / length_ljxi;
            H_dp(0, 1) = -(v_lj(0)*c - v_lj(2)*a) / length_lj / length_ljxi;
            H_dp(0, 2) = -(v_lj(1)*a - v_lj(0)*b) / length_lj / length_ljxi;

            rl(0) = -d_e;

            SO3 Rk = obs.curr_R_l_e;
            SO3 Rk_1 = obs.last_R_l_e;
            Triple pk = obs.curr_t_l_e;
            Triple pk_1 = obs.last_t_l_e;
            if (lidarproc->estimate_extrinsic)
            {
                if (!first_odo)
                {
                    H_px.block<3, 3>(0, 6 * id + 6) = skewSymmetric(Rk_1.transpose()*(Rk*obs.cornerFeature.currCornerPointCloud[i] + pk - pk_1));
                    H_px.block<3, 3>(0, 6 * id + 3 + 6) = -Rk_1.transpose();
                }
                H_px.block<3, 3>(0, 6 * id + 6 + 6) = -Rk_1.transpose()*Rk*skewSymmetric(obs.cornerFeature.currCornerPointCloud[i]);
                H_px.block<3, 3>(0, 6 * id + 9 + 6) = Rk_1.transpose();
            }
            else
            {
                if (!first_odo)
                {
                    H_px.block<3, 3>(0, 6 * id) = skewSymmetric(Rk_1.transpose()*(Rk*obs.cornerFeature.currCornerPointCloud[i] + pk - pk_1));
                    H_px.block<3, 3>(0, 6 * id + 3) = -Rk_1.transpose();
                }
                H_px.block<3, 3>(0, 6 * id + 6) = -Rk_1.transpose()*Rk*skewSymmetric(obs.cornerFeature.currCornerPointCloud[i]);
                H_px.block<3, 3>(0, 6 * id + 9) = Rk_1.transpose();
            }

            corner_H.row(i) = (H_dp * H_px).row(0);
            corner_r(i) = rl(0);
            
        }
        
    }


    for (int j = 0; j < obs.surfFeature.currSurfPointCloud.size(); j++)
    {

        ///< initialize the H matrix
        Matrix H_dp = Matrix::Zero(1, 3);
        Matrix H_px = Matrix::Zero(3, state_num);
        ///< initialize the r matrix
        Vector rl = Vector::Zero(1, 1);

        ///< calculate residual of observation(distance between point and plane)
        Triple v_jl = obs.surfFeature.correspondSurfPointCloudB[j] - obs.surfFeature.correspondSurfPointCloudA[j];
        Triple v_jm = obs.surfFeature.correspondSurfPointCloudC[j] - obs.surfFeature.correspondSurfPointCloudA[j];
        Triple v_ji = obs.surfFeature.currSurfPointCloud_inlast[j] - obs.surfFeature.correspondSurfPointCloudA[j];
        Triple vj_lm = v_jl.cross(v_jm);
        Triple up =Triple(vj_lm(0)*v_ji(0), vj_lm(1)*v_ji(1), vj_lm(2)*v_ji(2));
        double dj_lm = sqrt(vj_lm(0)*vj_lm(0)+ vj_lm(1)*vj_lm(1)+ vj_lm(2)*vj_lm(2));
        double d_p = sqrt(up(0)*up(0)+ up(1)*up(1)+ up(2)*up(2))/ dj_lm;

        double a = v_jl(1)*v_jm(2) - v_jl(2)*v_jm(1);
        double b = v_jl(2)*v_jm(0) - v_jl(0)*v_jm(2);
        double c = v_jl(0)*v_jm(1) - v_jl(1)*v_jm(0);

        H_dp(0, 0) = -a * a*v_ji(0) / d_p / dj_lm / dj_lm;
        H_dp(0, 1) = -b * b*v_ji(1) / d_p / dj_lm / dj_lm;
        H_dp(0, 2) = -c * c*v_ji(2) / d_p / dj_lm / dj_lm;

        rl(0) = -d_p;

        SO3 Rk = obs.curr_R_l_e;
        SO3 Rk_1 = obs.last_R_l_e;
        Triple pk = obs.curr_t_l_e;
        Triple pk_1 = obs.last_t_l_e;
        if (lidarproc->estimate_extrinsic)
        {
            if (!first_odo)
            {
                H_px.block<3, 3>(0, 6 * id + 6) = skewSymmetric(Rk_1.transpose()*(Rk*obs.surfFeature.currSurfPointCloud[j] + pk - pk_1));
                H_px.block<3, 3>(0, 6 * id + 3 + 6) = -Rk_1.transpose();
            }
            H_px.block<3, 3>(0, 6 * id + 6 + 6) = -Rk_1.transpose()*Rk*skewSymmetric(obs.surfFeature.currSurfPointCloud[j]);
            H_px.block<3, 3>(0, 6 * id + 9 + 6) = Rk_1.transpose();
        }
        else
        {
            if (!first_odo)
            {
                H_px.block<3, 3>(0, 6 * id) = skewSymmetric(Rk_1.transpose()*(Rk*obs.surfFeature.currSurfPointCloud[j] + pk - pk_1));
                H_px.block<3, 3>(0, 6 * id + 3) = -Rk_1.transpose();
            }
            H_px.block<3, 3>(0, 6 * id + 6) = -Rk_1.transpose()*Rk*skewSymmetric(obs.surfFeature.currSurfPointCloud[j]);
            H_px.block<3, 3>(0, 6 * id + 9) = Rk_1.transpose();
        }


        surf_H.row(j) = (H_dp * H_px).row(0);
        surf_r(j) = rl(0);
    }
    ///< if need to screen out some outpoints

    first_odo = false;

    H.resize(corner_H.rows() + surf_H.rows(), corner_H.cols());
    r.resize(corner_r.size() + surf_r.size());
    H << corner_H,
        surf_H;
    r << corner_r,
        surf_r;
    
}

void hwa_lidar::lidar_base::lidarMeasurementJacobian(lidarOdometryObs obs, Matrix &H, Vector &r,int project_id, int id, bool use_3d, float ther)
{
    //cout << "lidarodometry" << endl;
    //cout << "corner points:" << obs.cornerFeature.currCornerPointCloud.size() << endl;
    //cout << "surf points:" << obs.surfFeature.currSurfPointCloud.size() << endl;
    //cout << "lidar size:" << lidar_states.size() << endl;

    int corner_obsnum = obs.cornerFeature.currCornerPointCloud.size();
    int surf_obsnum = obs.surfFeature.currSurfPointCloud.size();
    int state_num = 0;
    if(lidarproc->estimate_extrinsic)
    {
        state_num = lidar_states.size() * 6 + 6;
    }
    else
    {
        state_num = lidar_states.size() * 6;
    }

    if (use_3d)
        corner_obsnum = corner_obsnum * 3;

    Matrix corner_H = Matrix::Zero(corner_obsnum, state_num);
    Vector corner_r = Vector::Zero(corner_obsnum);
    Matrix surf_H = Matrix::Zero(surf_obsnum, state_num);
    Vector surf_r = Vector::Zero(surf_obsnum);

    for (int i = 0; i < obs.cornerFeature.currCornerPointCloud.size(); i++)
    {
        if (use_3d)
        {
            ///< initialize the H matrix
            Matrix H_dp = Matrix::Zero(3, 3);
            Matrix H_px = Matrix::Zero(3, state_num);
            ///< initialize the r matrix
            Vector rl = Vector::Zero(3);

            ///< calculate the residual of observation(distance between point and line)
            Triple v_lj = obs.cornerFeature.correspondCornerPointCloudA[i] - obs.cornerFeature.correspondCornerPointCloudB[i];
            Triple v_li = obs.cornerFeature.currCornerPointCloud_inlast[i] - obs.cornerFeature.correspondCornerPointCloudB[i];
            Triple v_ji = obs.cornerFeature.currCornerPointCloud_inlast[i] - obs.cornerFeature.correspondCornerPointCloudA[i];
            Triple vjl_i = v_ji.cross(v_li);
            Triple vl_ji = v_lj.cross(v_li);
            double length_lj = sqrt(v_lj(0)*v_lj(0) + v_lj(1)*v_lj(1) + v_lj(2)*v_lj(2));;

            //这里的正负有待商榷
            H_dp(0, 1) = -v_lj(2) / length_lj;
            H_dp(0, 2) = v_lj(1) / length_lj;
            H_dp(1, 0) = v_lj(2) / length_lj;
            H_dp(1, 2) = -v_lj(0) / length_lj;
            H_dp(2, 0) = -v_lj(1) / length_lj;
            H_dp(2, 1) = v_lj(0) / length_lj;


            rl(0) = vl_ji.x() / length_lj;
            rl(1) = vl_ji.y() / length_lj;
            rl(2) = vl_ji.z() / length_lj;

            SO3 Rk = obs.curr_R_l_e;
            SO3 Rk_1 = obs.last_R_l_e;
            Triple pk = obs.curr_t_l_e;
            Triple pk_1 = obs.last_t_l_e;

            //不管什么时候都成立
            if (lidarproc->estimate_extrinsic)
            {
                H_px.block<3, 3>(0, 6 * project_id + 6) = skewSymmetric(Rk_1.transpose()*(Rk*obs.cornerFeature.currCornerPointCloud[i] + pk - pk_1));
                H_px.block<3, 3>(0, 6 * project_id + 3 + 6) = -Rk_1.transpose();

                H_px.block<3, 3>(0, 6 * id + 6) = -Rk_1.transpose()*Rk*skewSymmetric(obs.cornerFeature.currCornerPointCloud[i]);
                H_px.block<3, 3>(0, 6 * id + 3 + 6) = Rk_1.transpose();
            }
            else
            {
                H_px.block<3, 3>(0, 6 * project_id) = skewSymmetric(Rk_1.transpose()*(Rk*obs.cornerFeature.currCornerPointCloud[i] + pk - pk_1));
                H_px.block<3, 3>(0, 6 * project_id + 3) = -Rk_1.transpose();

                H_px.block<3, 3>(0, 6 * id) = -Rk_1.transpose()*Rk*skewSymmetric(obs.cornerFeature.currCornerPointCloud[i]);
                H_px.block<3, 3>(0, 6 * id + 3) = Rk_1.transpose();
            }

            Matrix temp_H = H_dp * H_px;
            corner_H.row(i * 3) = temp_H.row(0);
            corner_H.row(i * 3 + 1) = temp_H.row(1);
            corner_H.row(i * 3 + 2) = temp_H.row(2);
            corner_r(i * 3) = rl(0);
            corner_r(i * 3 + 1) = rl(1);
            corner_r(i * 3 + 2) = rl(2);
        }
        else
        {
            ///< initialize the H matrix
            Matrix H_dp = Matrix::Zero(1, 3);
            Matrix H_px = Matrix::Zero(3, state_num);
            ///< initialize the r matrix
            Vector rl = Vector::Zero(1);

            ///< calculate the residual of observation(distance between the point and line)
            Triple v_lj = obs.cornerFeature.correspondCornerPointCloudA[i] - obs.cornerFeature.correspondCornerPointCloudB[i];
            Triple v_li = obs.cornerFeature.currCornerPointCloud_inlast[i] - obs.cornerFeature.correspondCornerPointCloudB[i];
            Triple v_ljxi = v_lj.cross(v_li);
            double length_lj = sqrt(v_lj(0)*v_lj(0) + v_lj(1)*v_lj(1) + v_lj(2)*v_lj(2));
            double length_ljxi = sqrt(v_ljxi(0)*v_ljxi(0) + v_ljxi(1)*v_ljxi(1) + v_ljxi(2)*v_ljxi(2));
            double d_e = length_ljxi / length_lj;

            double a = v_lj(1)*v_li(2) - v_lj(2)*v_li(1);
            double b = v_lj(2)*v_li(0) - v_lj(0)*v_li(2);
            double c = v_lj(0)*v_li(1) - v_lj(1)*v_li(0);

            H_dp(0, 0) = (v_lj(2)*b - v_lj(1)*c) / length_lj / length_ljxi;
            H_dp(0, 1) = (v_lj(0)*c - v_lj(2)*a) / length_lj / length_ljxi;
            H_dp(0, 2) = (v_lj(1)*a - v_lj(0)*b) / length_lj / length_ljxi;

            rl(0) = d_e;

            SO3 Rk = obs.curr_R_l_e;
            SO3 Rk_1 = obs.last_R_l_e;
            Triple pk = obs.curr_t_l_e;
            Triple pk_1 = obs.last_t_l_e;

            if (lidarproc->estimate_extrinsic)
            {
                H_px.block<3, 3>(0, 6 * project_id + 6) = skewSymmetric(Rk_1.transpose()*(Rk*obs.cornerFeature.currCornerPointCloud[i] + pk - pk_1));
                H_px.block<3, 3>(0, 6 * project_id + 3 + 6) = -Rk_1.transpose();
                //                
                H_px.block<3, 3>(0, 6 * id + 6) = -Rk_1.transpose()*Rk*skewSymmetric(obs.cornerFeature.currCornerPointCloud[i]);
                H_px.block<3, 3>(0, 6 * id + 3 + 6) = Rk_1.transpose();
            }
            else
            {
                H_px.block<3, 3>(0, 6 * project_id) = skewSymmetric(Rk_1.transpose()*(Rk*obs.cornerFeature.currCornerPointCloud[i] + pk - pk_1));
                H_px.block<3, 3>(0, 6 * project_id + 3) = -Rk_1.transpose();
                //
                H_px.block<3, 3>(0, 6 * id) = -Rk_1.transpose()*Rk*skewSymmetric(obs.cornerFeature.currCornerPointCloud[i]);
                H_px.block<3, 3>(0, 6 * id + 3) = Rk_1.transpose();
            }


            corner_H.row(i) = (H_dp * H_px).row(0);
            corner_r(i) = rl(0);

        }

    }
    ///< if need to screen out some outpoints

    for (int j = 0; j < obs.surfFeature.currSurfPointCloud.size(); j++)
    {
        ///< initialize the H matrix
        Matrix H_dp = Matrix::Zero(1, 3);
        Matrix H_px = Matrix::Zero(3, state_num);
        ///< initialize the r matrix
        Vector rl = Vector::Zero(1, 1);

        ///< calculate residual of observation(distance between point and plane)
        Triple v_jl = obs.surfFeature.correspondSurfPointCloudB[j] - obs.surfFeature.correspondSurfPointCloudA[j];
        Triple v_jm = obs.surfFeature.correspondSurfPointCloudC[j] - obs.surfFeature.correspondSurfPointCloudA[j];
        Triple v_ji = obs.surfFeature.currSurfPointCloud_inlast[j] - obs.surfFeature.correspondSurfPointCloudA[j];
        Triple vj_lm = v_jl.cross(v_jm);
        Triple up = Triple(vj_lm(0)*v_ji(0), vj_lm(1)*v_ji(1), vj_lm(2)*v_ji(2));
        double dj_lm = sqrt(vj_lm(0)*vj_lm(0) + vj_lm(1)*vj_lm(1) + vj_lm(2)*vj_lm(2));
        double d_p = sqrt(up(0)*up(0) + up(1)*up(1) + up(2)*up(2)) / dj_lm;

        double a = v_jl(1)*v_jm(2) - v_jl(2)*v_jm(1);
        double b = v_jl(2)*v_jm(0) - v_jl(0)*v_jm(2);
        double c = v_jl(0)*v_jm(1) - v_jl(1)*v_jm(0);

        H_dp(0, 0) = a * a*v_ji(0) / d_p / dj_lm / dj_lm;
        H_dp(0, 1) = b * b*v_ji(1) / d_p / dj_lm / dj_lm;
        H_dp(0, 2) = c * c*v_ji(2) / d_p / dj_lm / dj_lm;

        rl(0) = d_p;


        SO3 Rk = obs.curr_R_l_e;
        SO3 Rk_1 = obs.last_R_l_e;
        Triple pk = obs.curr_t_l_e;
        Triple pk_1 = obs.last_t_l_e;

        if (lidarproc->estimate_extrinsic)
        {
            H_px.block<3, 3>(0, 6 * project_id + 6) = skewSymmetric(Rk_1.transpose()*(Rk*obs.surfFeature.currSurfPointCloud[j] + pk - pk_1));
            H_px.block<3, 3>(0, 6 * project_id + 3 + 6) = -Rk_1.transpose();
                             
            H_px.block<3, 3>(0, 6 * id + 6) = -Rk_1.transpose()*Rk*skewSymmetric(obs.surfFeature.currSurfPointCloud[j]);
            H_px.block<3, 3>(0, 6 * id + 3 + 6) = Rk_1.transpose();
        }
        else
        {
            H_px.block<3, 3>(0, 6 * project_id) = skewSymmetric(Rk_1.transpose()*(Rk*obs.surfFeature.currSurfPointCloud[j] + pk - pk_1));
            H_px.block<3, 3>(0, 6 * project_id + 3) = -Rk_1.transpose();

            H_px.block<3, 3>(0, 6 * id) = -Rk_1.transpose()*Rk*skewSymmetric(obs.surfFeature.currSurfPointCloud[j]);
            H_px.block<3, 3>(0, 6 * id + 3) = Rk_1.transpose();
        }


        surf_H.row(j) = (H_dp * H_px).row(0);
        surf_r(j) = rl(0);
    
    }
    ///< if need to screen out some outpoints

    first_odo = false;

    H.resize(corner_H.rows() + surf_H.rows(), corner_H.cols());
    r.resize(corner_r.size() + surf_r.size());
    H << corner_H,
        surf_H;
    r << corner_r,
        surf_r;

}

void hwa_lidar::lidar_base::lidarMeasurementJacobian(lidarMappingObs obs, Matrix &H, Vector &r, int id,bool use_3d,float ther)
{
    int corner_obsnum = obs.cornerFeature.currCornerPointCloud.size();
    int surf_obsnum = obs.surfFeature.currSurfPointCloud.size();
    int state_num = 0;
    if (lidarproc->estimate_extrinsic)
    {
        state_num = lidar_states.size() * 6 + 6;
    }
    else
    {
        state_num = lidar_states.size() * 6;
    }


    Matrix corner_H = Matrix::Zero(corner_obsnum, state_num);
    Vector corner_r = Vector::Zero(corner_obsnum);
    Matrix surf_H = Matrix::Zero(surf_obsnum, state_num);
    Vector surf_r = Vector::Zero(surf_obsnum);

    SO3 Rk = obs.curr_R_l_e;
    Triple pk = obs.curr_t_l_e;
    SO3 Rk_1 = obs.last_R_l_e;
    Triple pk_1 = obs.last_t_l_e;


    for (int i = 0; i < obs.cornerFeature.currCornerPointCloud.size(); i++)
    {

        ///< initialize the H matrix
        Matrix H_dp = Matrix::Zero(1, 3);
        Matrix H_px = Matrix::Zero(3, state_num);
        ///< initialize the r matrix
        Vector rl = Vector::Zero(1);

        ///< calculate the residual of observation(distance between point and line)
        Triple v_lj = obs.cornerFeature.correspondCornerPointCloudA[i] - obs.cornerFeature.correspondCornerPointCloudB[i];
        Triple v_li = obs.cornerFeature.currCornerPointCloud_inlast[i] - obs.cornerFeature.correspondCornerPointCloudB[i];
        Triple v_ljxi = v_lj.cross(v_li);
        double length_lj = sqrt(v_lj(0)*v_lj(0) + v_lj(1)*v_lj(1) + v_lj(2)*v_lj(2));
        double length_ljxi = sqrt(v_ljxi(0)*v_ljxi(0) + v_ljxi(1)*v_ljxi(1) + v_ljxi(2)*v_ljxi(2));
        double d_e = length_ljxi / length_lj;

        double a = v_lj(1)*v_li(2) - v_lj(2)*v_li(1);
        double b = v_lj(2)*v_li(0) - v_lj(0)*v_li(2);
        double c = v_lj(0)*v_li(1) - v_lj(1)*v_li(0);

        H_dp(0, 0) = (v_lj(2)*b - v_lj(1)*c) / length_lj / length_ljxi;
        H_dp(0, 1) = (v_lj(0)*c - v_lj(2)*a) / length_lj / length_ljxi;
        H_dp(0, 2) = (v_lj(1)*a - v_lj(0)*b) / length_lj / length_ljxi;
        rl(0) = d_e;

        if (!first_map)//!first_map)
        {
            if (lidarproc->estimate_extrinsic)
            {
                H_px.block<3, 3>(0, 6) = skewSymmetric(Rk_1.transpose()*(Rk*obs.cornerFeature.currCornerPointCloud[i] + pk - pk_1));
                H_px.block<3, 3>(0, 9) = -Rk_1.transpose();
            }
            else
            {
                H_px.block<3, 3>(0, 0) = skewSymmetric(Rk_1.transpose()*(Rk*obs.cornerFeature.currCornerPointCloud[i] + pk - pk_1));
                H_px.block<3, 3>(0, 3) = -Rk_1.transpose();
            }
        }


        H_px.block<3, 3>(0, 6 * id) = -Rk_1.transpose()*Rk*skewSymmetric(obs.cornerFeature.currCornerPointCloud[i]);
        H_px.block<3, 3>(0, 6 * id + 3) = Rk_1.transpose();
        //H_px.block<3, 3>(0, 6+6) = -Rk_1.transpose()*Rk*skewSymmetric(obs.cornerFeature.currCornerPointCloud[i]);
        //H_px.block<3, 3>(0, 6+6+3) = Rk_1.transpose();


        corner_H.row(i) = (H_dp * H_px).row(0);
        corner_r(i) = rl(0);


    }


    for (int j = 0; j < obs.surfFeature.currSurfPointCloud.size(); j++)
    {
        ///< initialize the H matrix
        Matrix H_dp = Matrix::Zero(1, 3);
        Matrix H_px = Matrix::Zero(3, state_num);
        ///< initialize the r matrix
        Vector rl = Vector::Zero(1);
        ///< calculate residual of observation(distance between point and plane)
        rl(0) = (obs.surfFeature.norm[j].dot(obs.surfFeature.currSurfPointCloud_inlast[j]) + obs.surfFeature.negative_OA_dot_norm[j]);

        H_dp(0, 0) = obs.surfFeature.norm[j](0);
        H_dp(0, 1) = obs.surfFeature.norm[j](1);
        H_dp(0, 2) = obs.surfFeature.norm[j](2);


        if (!first_map)//!first_map)
        {
            if (lidarproc->estimate_extrinsic)
            {
                H_px.block<3, 3>(0, 6) = skewSymmetric(Rk_1.transpose()*(Rk*obs.surfFeature.currSurfPointCloud[j] + pk - pk_1));
                H_px.block<3, 3>(0, 9) = -Rk_1.transpose();
            }
            else
            {
                H_px.block<3, 3>(0, 0) = skewSymmetric(Rk_1.transpose()*(Rk*obs.surfFeature.currSurfPointCloud[j] + pk - pk_1));
                H_px.block<3, 3>(0, 3) = -Rk_1.transpose();
            }
        }

        H_px.block<3, 3>(0, 6 * id) = -Rk_1.transpose()*Rk*skewSymmetric(obs.surfFeature.currSurfPointCloud[j]);
        H_px.block<3, 3>(0, 6 * id + 3) = Rk_1.transpose();

        surf_H.row(j) = (H_dp * H_px).row(0);
        surf_r(j) = rl(0);
    }


    first_map = false;

    H.resize(surf_H.rows() + corner_H.rows(), surf_H.cols());
    r.resize(surf_r.size() + corner_r.size());

    H << surf_H,
        corner_H;
    r << surf_r,
        corner_r;
}


void hwa_lidar::lidar_base::lidarMeasurementJacobian_priormap(lidarMappingObs obs, Matrix &H, Vector &r, bool use_3d, float ther)
{
    first_map = false;
    int corner_obsnum = obs.cornerFeature.currCornerPointCloud.size();
    int surf_obsnum = obs.surfFeature.currSurfPointCloud.size();
    int state_num = 0;
    if (lidarproc->estimate_extrinsic)
    {
        state_num = lidar_states.size() * 6 + 6;
    }
    else
    {
        state_num = lidar_states.size() * 6;
    }
    if (lidar_states.size() <=1)
    {
        cout << "window_size:" << window_size << endl;
        cout << "lidar states:" << lidar_states.size() << endl;
        cout << "there is something wrong with lidar states" << endl;
        cin.get();
    }
    //暂时没有用到3d，不考虑，后续需要使用时需要检查
    if (use_3d)
        corner_obsnum = corner_obsnum * 3;

    Matrix corner_H = Matrix::Zero(corner_obsnum, state_num);
    Vector corner_r = Vector::Zero(corner_obsnum);
    Matrix surf_H = Matrix::Zero(surf_obsnum, state_num);
    Vector surf_r = Vector::Zero(surf_obsnum);

    SO3 Rk = obs.curr_R_l_e;
    Triple pk = obs.curr_t_l_e;


    for (int i = 0; i < obs.cornerFeature.currCornerPointCloud.size(); i++)
    {
        if (use_3d)
        {
            ///< initialize the H matrix
            Matrix H_dp = Matrix::Zero(3, 3);
            Matrix H_px = Matrix::Zero(3, state_num);
            ///< initialize the r matrix
            Vector rl = Vector::Zero(3);

            ///< calculate residual of observation(distance between point and line)
            Triple v_lj = obs.cornerFeature.correspondCornerPointCloudA[i] - obs.cornerFeature.correspondCornerPointCloudB[i];
            Triple v_li = obs.cornerFeature.currCornerPointCloud_inlast[i] - obs.cornerFeature.correspondCornerPointCloudB[i];
            Triple v_ji = obs.cornerFeature.currCornerPointCloud_inlast[i] - obs.cornerFeature.correspondCornerPointCloudA[i];
            Triple vjl_i = v_ji.cross(v_li);
            Triple vl_ji = v_lj.cross(v_li);
            double length_lj = sqrt(v_lj(0)*v_lj(0) + v_lj(1)*v_lj(1) + v_lj(2)*v_lj(2));;

            H_dp(0, 1) = -v_lj(2) / length_lj;
            H_dp(0, 2) = v_lj(1) / length_lj;
            H_dp(1, 0) = v_lj(2) / length_lj;
            H_dp(1, 2) = -v_lj(0) / length_lj;
            H_dp(2, 0) = -v_lj(1) / length_lj;
            H_dp(2, 1) = v_lj(0) / length_lj;

            rl(0) = vl_ji.x() / length_lj;
            rl(1) = vl_ji.y() / length_lj;
            rl(2) = vl_ji.z() / length_lj;

            //Triple p_last = R_tolast*obs.cornerFeature.currCornerPointCloud[i]+t_tolast;
            //if (first_map)
            //{
            //    H_px.block<3, 3>(0, 0) = SO3::Zero();
            //    H_px.block<3, 3>(0, 3) = SO3::Zero();
            //}
            //else
            //{
            //    H_px.block<3, 3>(0, 0) = -R_w_e.transpose()*Rk_1*skewSymmetric(p_last);
            //    H_px.block<3, 3>(0, 3) = R_w_e.transpose();
            //    /*H_px.block<3, 3>(0, 0) = SO3::Zero();
            //    H_px.block<3, 3>(0, 3) = SO3::Zero();*/
            //}

            /*if (first_map)
            {
                H_px.block<3, 3>(0, 0) = SO3::Zero();
                H_px.block<3, 3>(0, 3) = SO3::Zero();
            }
            else
            {
                H_px.block<3, 3>(0, 0) = skewSymmetric(Rk_1.transpose()*Rk*obs.cornerFeature.currCornerPointCloud[i]) + skewSymmetric(R_w_e.transpose()*(pk - pk_1));
                H_px.block<3, 3>(0, 3) = -Rk_1.transpose();
            }*/
            H_px.block<3, 3>(0, state_num-6) = -Rk*skewSymmetric(obs.cornerFeature.currCornerPointCloud[i]);
            H_px.block<3, 3>(0, state_num-3) = SO3::Identity();

            Matrix temp_H;
            temp_H= H_dp * H_px;
            corner_H.row(i * 3) = temp_H.row(0);
            corner_H.row(i * 3 + 1) = temp_H.row(1);
            corner_H.row(i * 3 + 2) = temp_H.row(2);
            corner_r(i * 3) = rl(0);
            corner_r(i * 3 + 1) = rl(1);
            corner_r(i * 3 + 2) = rl(2);
        }
        else
        {
            ///< initialize the H matrix
            Matrix H_dp = Matrix::Zero(1, 3);
            Matrix H_px = Matrix::Zero(3, state_num);
            ///< initialize the r matrix
            Vector rl = Vector::Zero(1);

            ///< calculate the residual of observation(distance between point and line)
            Triple v_lj = obs.cornerFeature.correspondCornerPointCloudA[i] - obs.cornerFeature.correspondCornerPointCloudB[i];
            Triple v_li = obs.cornerFeature.currCornerPointCloud_inlast[i] - obs.cornerFeature.correspondCornerPointCloudB[i];
            Triple v_ljxi = v_lj.cross(v_li);

            double length_lj = sqrt(v_lj(0)*v_lj(0) + v_lj(1)*v_lj(1) + v_lj(2)*v_lj(2));
            double length_ljxi = sqrt(v_ljxi(0)*v_ljxi(0) + v_ljxi(1)*v_ljxi(1) + v_ljxi(2)*v_ljxi(2));
            double d_e = length_ljxi / length_lj;

            double a = v_lj(1)*v_li(2) - v_lj(2)*v_li(1);
            double b = v_lj(2)*v_li(0) - v_lj(0)*v_li(2);
            double c = v_lj(0)*v_li(1) - v_lj(1)*v_li(0);

            H_dp(0, 0) = (v_lj(2)*b - v_lj(1)*c) / length_lj / length_ljxi;
            H_dp(0, 1) = (v_lj(0)*c - v_lj(2)*a) / length_lj / length_ljxi;
            H_dp(0, 2) = (v_lj(1)*a - v_lj(0)*b) / length_lj / length_ljxi;

            rl(0) = d_e;



            if (lidarproc->estimate_extrinsic)
            {
                H_px.block<3, 3>(0, state_num - 6) = -Rk * skewSymmetric(obs.cornerFeature.currCornerPointCloud[i]);
                H_px.block<3, 3>(0, state_num - 3) = SO3::Identity();
            }
            else
            {
                H_px.block<3, 3>(0, state_num - 6) = -Rk * skewSymmetric(obs.cornerFeature.currCornerPointCloud[i]);
                H_px.block<3, 3>(0, state_num - 3) = SO3::Identity();
            }

            corner_H.row(i) = H_dp * H_px;
            corner_r(i) = rl(0);
            if (fabs(rl(0)) < 1e-6)
            {
                cout << "rl max:" << rl(0) << endl;
            }
        }
    }


    for (int j = 0; j < obs.surfFeature.currSurfPointCloud.size(); j++)
    {
        ///< initialize the H matrix
        Matrix H_dp = Matrix::Zero(1, 3);
        Matrix H_px = Matrix::Zero(3, state_num);
        ///< initialize the r matrix
        Vector rl = Vector::Zero(1);
        ///< calculate residual of observation(distance between point and plane)
        rl(0) = (obs.surfFeature.norm[j].dot(obs.surfFeature.currSurfPointCloud_inlast[j]) + obs.surfFeature.negative_OA_dot_norm[j]);
        //cout << obs.surfFeature.norm[j].dot(obs.surfFeature.currSurfPointCloud_inlast[j]) << endl;
        //cout << obs.surfFeature.negative_OA_dot_norm[j] << endl;
        H_dp(0, 0) = obs.surfFeature.norm[j](0);
        H_dp(0, 1) = obs.surfFeature.norm[j](1);
        H_dp(0, 2) = obs.surfFeature.norm[j](2);

        //H_px.block<3, 3>(0, state_num - 6) = -Rk * skewSymmetric(obs.surfFeature.currSurfPointCloud[j]);
        //H_px.block<3, 3>(0, state_num - 3) = SO3::Identity();
        if (lidarproc->estimate_extrinsic)
        {
            H_px.block<3, 3>(0, state_num - 6) = -Rk * skewSymmetric(obs.surfFeature.currSurfPointCloud[j]);
            H_px.block<3, 3>(0, state_num - 3) = SO3::Identity();
        }
        else
        {
            H_px.block<3, 3>(0, state_num - 6) = -Rk * skewSymmetric(obs.surfFeature.currSurfPointCloud[j]);
            H_px.block<3, 3>(0, state_num - 3) = SO3::Identity();
        }
        //Huber(H_px, rl, ther);

        surf_H.row(j) = H_dp * H_px;
        surf_r(j) = rl(0);
        if (fabs(rl(0)) < 1e-6)
        {
            cout << "rl max:" << rl(0) << endl;
        }

    }


    first_map = false;

    H.resize(surf_H.rows() + corner_H.rows(), surf_H.cols());
    r.resize(surf_r.size() + corner_r.size());

    H << surf_H,
        corner_H;
    r << surf_r,
        corner_r;

}