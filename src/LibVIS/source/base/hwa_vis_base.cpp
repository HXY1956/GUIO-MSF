#include "hwa_vis_base.h"
#include "hwa_set_proc.h"

hwa_vis::vis_base::vis_base(hwa_set::set_base* _set, int cam_group_id) :cam_state_id(0), cam_next_id(0)
{
    processer = dynamic_cast<set_vis*>(_set)->processer();
    max_camstate_size = dynamic_cast<set_vis*>(_set)->max_cam_state_size(cam_group_id);
    _MinParallex = dynamic_cast<set_vis*>(_set)->minparallex(cam_group_id);
    R_cam0_imu = dynamic_cast<set_vis*>(_set)->R_cam0_imu(cam_group_id);
    t_cam0_imu = dynamic_cast<set_vis*>(_set)->t_cam0_imu(cam_group_id);
    T_cam0_imu = dynamic_cast<set_vis*>(_set)->T_cam0_imu(cam_group_id);
    R_cam0_cam1 = dynamic_cast<set_vis*>(_set)->R_cam0_cam1(cam_group_id);
    t_cam0_cam1 = dynamic_cast<set_vis*>(_set)->t_cam0_cam1(cam_group_id);
    T_cam0_cam1 = dynamic_cast<set_vis*>(_set)->T_cam0_cam1(cam_group_id);
    vis_feature::read_Tc0c1(T_cam0_cam1);
    usingstereorecity = dynamic_cast<set_vis*>(_set)->usingstereorecify(cam_group_id);
    cam0_intrinsics = dynamic_cast<set_vis*>(_set)->cam0_intrinsics(cam_group_id);
    num_of_cam = dynamic_cast<set_vis*>(_set)->num_of_cam(cam_group_id);
    feature_observation_noise = dynamic_cast<set_vis*>(_set)->feature_observation_noise(cam_group_id);
    feature_observation_noise *= feature_observation_noise;
    Bgs.resize(max_camstate_size, Triple::Zero());
    Bas.resize(max_camstate_size, Triple::Zero());
    if (num_of_cam == 1) stereo = false;
    else stereo = true;
    switch (processer) {
    case CPU:
        if (!stereo)
            imgproc = std::make_unique<vis_mono_lk_cpu>(_set, cam_group_id);
        else
            imgproc = std::make_unique<vis_stereo_lk_cpu>(_set, cam_group_id);
        break;
    case GPU:
        if (!stereo)
            imgproc = std::make_unique<vis_mono_lk_gpu>(_set, cam_group_id);
        else
            imgproc = std::make_unique<vis_stereo_lk_gpu>(_set, cam_group_id);
        break;
    }
    clone = dynamic_cast<set_vis*>(_set)->clone_type();
    readChisquare_test();
    if (usingstereorecity)
    {
        T_cam0_cam1 = imgproc->T_cam0_cam1;
        R_cam0_cam1 = imgproc->R_cam0_cam1;
        t_cam0_cam1 = imgproc->t_cam0_cam1;
        T_cam0_imu = imgproc->T_cam0_imu;
        R_cam0_imu = imgproc->R_cam0_imu;
        t_cam0_imu = imgproc->t_cam0_imu;
    }
    if (imgproc->estimate_extrinsic)
        ex_param_num += 6;
    if (num_of_cam == 2 && imgproc->estimate_extrinsic_allcam)
        ex_param_num += 6;
    if (imgproc->estimate_t)
        ex_param_num += 1;
    if (num_of_cam == 2 && imgproc->estimate_t_allcam)
        ex_param_num += 1;
};

hwa_vis::vis_base::~vis_base()
{
}

void hwa_vis::vis_base::readChisquare_test()
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

void hwa_vis::vis_base::PnP(Eigen::Quaterniond& q, Triple& t) {

}

bool hwa_vis::vis_base::keyframeCheck()
{
    auto cur_cam_id = cam_state_id;
    int curr_feature_num = map_server.size();
    int tracked_feature_num = 0;
    double MIN_PARALLAX = _MinParallex / cam0_intrinsics(0);
    double parallax = 0;
    double parallax_sum = 0;
    int parallax_num = 0;
    int last_track_num = 0;
    for (auto pts : _pointcloud.features)
    {
        int feature_id = pts.id;
        auto iter = map_server.find(feature_id);
        if (iter != map_server.end())
        {
            Eigen::Vector4d uv_cur = Eigen::Vector4d(pts.cam0_point.x, pts.cam0_point.y, pts.cam1_point.x, pts.cam1_point.y);
            ++tracked_feature_num;
            auto obser = iter->second.observations.rbegin();
            Eigen::Vector4d uv_pre = obser->second;
            double du = uv_cur(0) - uv_pre(0);
            double dv = uv_cur(1) - uv_pre(1);
            parallax = sqrt(du * du + dv * dv);
            parallax_sum += parallax;
            parallax_num++;
        }
    }

    tracking_rate =
        static_cast<double>(tracked_feature_num) /
        static_cast<double>(curr_feature_num);

    if (parallax_num == 0)
    {
        nonFrame_num = 0;
        isKeyFrame = true;
        KeyFrameBuffer.push_back(cur_cam_id);
        return true;
    }

    double ans = parallax_sum / parallax_num;
    if (tracked_feature_num < 20 || nonFrame_num >= 30 || ans >= MIN_PARALLAX)
    {
        std::cout << "Cam-" << cam_state_id << ": KeyFrame " << ": parallax: " << ans << " Feature Num: " << tracked_feature_num << std::endl;
        nonFrame_num = 0;
        isKeyFrame = true;
        KeyFrameBuffer.push_back(cur_cam_id);
        return true;
    }
    else
    {
        nonFrame_num++;
        nonKeyFrameBuffer.push_back(cur_cam_id);
        isKeyFrame = false;
        return false;
    }
    return true;
}

void hwa_vis::vis_base::measurementJacobianEX(
    const CamStateIDType& cam_state_id,
    const FeatureIDType& feature_id,
    Eigen::Matrix<double, 4, 6>& H_x, Eigen::Matrix<double, 4, 6>& H_x_ex, Eigen::Matrix<double, 4, 3>& H_f, Eigen::Vector4d& r)
{
    const CamState& cam_state = cam_states[cam_state_id];
    const CamState& imu_state = imu_states[cam_state_id];
    const vis_feature& feature = map_server[feature_id];


    //SO3 R_w_c0 = cam_state.orientation.toRotationMatrix();
    SO3 R_w_c0 = cam_state.orientation.toRotationMatrix().transpose();
    const Triple& t_c0_w = cam_state.position;
    const Triple& t_i_w = imu_state.position;

    SO3 R_c0_i = T_cam0_imu.linear();
    SO3 R_i_c0 = R_c0_i.transpose();
    SO3 R_c0_c1 = T_cam0_cam1.linear();
    Triple t_c0_c1 = T_cam0_cam1.translation();
    SO3 R_w_c1 = T_cam0_cam1.linear() * R_w_c0;
    Triple t_c1_w = t_c0_w - R_w_c1.transpose() * t_c0_c1;

    const Triple& p_w = feature.position;
    const Eigen::Vector4d& z = feature.observations.find(cam_state_id)->second;

    Triple p_c0 = R_w_c0 * (p_w - t_c0_w);
    Triple p_c1 = R_w_c1 * (p_w - t_c1_w);

    Eigen::Matrix<double, 4, 3> dz_dpc0 = Eigen::Matrix<double, 4, 3>::Zero();
    dz_dpc0(0, 0) = 1 / p_c0(2);
    dz_dpc0(1, 1) = 1 / p_c0(2);
    dz_dpc0(0, 2) = -p_c0(0) / (p_c0(2) * p_c0(2));
    dz_dpc0(1, 2) = -p_c0(1) / (p_c0(2) * p_c0(2));

    Eigen::Matrix<double, 4, 3> dz_dpc1 = Eigen::Matrix<double, 4, 3>::Zero();
    dz_dpc1(2, 0) = 1 / p_c1(2);
    dz_dpc1(3, 1) = 1 / p_c1(2);
    dz_dpc1(2, 2) = -p_c1(0) / (p_c1(2) * p_c1(2));
    dz_dpc1(3, 2) = -p_c1(1) / (p_c1(2) * p_c1(2));

    Eigen::Matrix<double, 3, 6> dpc0_dxc = Eigen::Matrix<double, 3, 6>::Zero();
    Eigen::Matrix<double, 3, 6> dpc1_dxc = Eigen::Matrix<double, 3, 6>::Zero();

    Eigen::Matrix<double, 3, 6> dpc0_dxc_ex = Eigen::Matrix<double, 3, 6>::Zero();
    Eigen::Matrix<double, 3, 6> dpc1_dxc_ex = Eigen::Matrix<double, 3, 6>::Zero();

    if (clone == CAMERA) {
        if (vfusetype == "Normal") {

            dpc0_dxc.leftCols(3) = R_w_c0 * skew(p_w - t_c0_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            dpc1_dxc.leftCols(3) = R_w_c1 * skew(p_w - t_c0_w);
            dpc1_dxc.rightCols(3) = R_w_c1;

            r = z - Eigen::Vector4d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2),
                p_c1(0) / p_c1(2), p_c1(1) / p_c1(2));
        }
        else if (vfusetype == "InEKF" || vfusetype == "I_InEKF") {
            dpc0_dxc.leftCols(3) = -R_w_c0 * skew(p_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            dpc1_dxc.leftCols(3) = -R_w_c1 * skew(p_w + R_w_c1.transpose() * T_cam0_cam1.translation());
            dpc1_dxc.rightCols(3) = R_w_c1;

            r = z - Eigen::Vector4d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2),
                p_c1(0) / p_c1(2), p_c1(1) / p_c1(2));
        }
    }
    else if (clone == IMU) {
        if (vfusetype == "Normal") {

            dpc0_dxc.leftCols(3) = R_w_c0 * skew(p_w - t_i_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            dpc1_dxc.leftCols(3) = R_w_c1 * skew(p_w - t_i_w);
            dpc1_dxc.rightCols(3) = R_w_c1;

            dpc0_dxc_ex.leftCols(3) = -skew(R_w_c0 * (p_w - t_c0_w));
            dpc0_dxc_ex.rightCols(3) = -R_i_c0;

            dpc1_dxc_ex.leftCols(3) = -skew(R_w_c1 * (p_w - t_c1_w)) + skew(t_c0_c1) * R_c0_c1;
            dpc1_dxc_ex.rightCols(3) = -R_c0_c1 * R_i_c0;

            r = z - Eigen::Vector4d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2),
                p_c1(0) / p_c1(2), p_c1(1) / p_c1(2));
        }
        else if (vfusetype == "InEKF" || vfusetype == "I_InEKF") {
            dpc0_dxc.leftCols(3) = -R_w_c0 * skew(p_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            dpc1_dxc.leftCols(3) = -R_w_c1 * skew(p_w + R_w_c1.transpose() * T_cam0_cam1.translation());
            dpc1_dxc.rightCols(3) = R_w_c1;

            dpc0_dxc_ex.leftCols(3) = -skew(R_w_c0 * (p_w - t_c0_w));
            dpc0_dxc_ex.rightCols(3) = -R_i_c0;

            dpc1_dxc_ex.leftCols(3) = -skew(R_w_c1 * (p_w - t_c1_w)) + skew(t_c0_c1) * R_c0_c1;
            dpc1_dxc_ex.rightCols(3) = -R_c0_c1 * R_i_c0;

            r = z - Eigen::Vector4d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2),
                p_c1(0) / p_c1(2), p_c1(1) / p_c1(2));
        }

    }


    SO3 dpc0_dpg = R_w_c0;
    SO3 dpc1_dpg = R_w_c1;

    H_x = dz_dpc0 * dpc0_dxc + dz_dpc1 * dpc1_dxc;
    H_f = dz_dpc0 * dpc0_dpg + dz_dpc1 * dpc1_dpg;
    H_x_ex = dz_dpc0 * dpc0_dxc_ex + dz_dpc1 * dpc1_dxc_ex;

    //if (vfusetype == "Normal") {
    //    //OC
    //    Eigen::Matrix<double, 4, 6> A = H_x;
    //    Eigen::Matrix<double, 6, 1> u = Eigen::Matrix<double, 6, 1>::Zero();
    //    u.block<3, 1>(0, 0) = cam_state.orientation_null.toRotationMatrix() * cam_state.gravity;
    //    u.block<3, 1>(3, 0) = skew(p_w - cam_state.position_null) * cam_state.gravity;
    //    H_x = A - A * u * (u.transpose() * u).inverse() * u.transpose();
    //    H_f = -H_x.block<4, 3>(0, 3);
    //}

}


void hwa_vis::vis_base::measurementJacobian(
    const CamStateIDType& cam_state_id,
    const FeatureIDType& feature_id,
    Eigen::Matrix<double, 4, 6>& H_x, Eigen::Matrix<double, 4, 3>& H_f, Eigen::Vector4d& r)
{
    const CamState& cam_state = cam_states[cam_state_id];
    const CamState& imu_state = imu_states[cam_state_id];
    const vis_feature& feature = map_server[feature_id];


    //SO3 R_w_c0 = cam_state.orientation.toRotationMatrix();
    SO3 R_w_c0 = cam_state.orientation.toRotationMatrix().transpose();
    const Triple& t_c0_w = cam_state.position;
    const Triple& t_i_w = imu_state.position;


    SO3 R_c0_c1 = T_cam0_cam1.linear();
    SO3 R_w_c1 = T_cam0_cam1.linear() * R_w_c0;
    Triple t_c1_w = t_c0_w - R_w_c1.transpose() * T_cam0_cam1.translation();

    const Triple& p_w = feature.position;
    const Eigen::Vector4d& z = feature.observations.find(cam_state_id)->second;

    Triple p_c0 = R_w_c0 * (p_w - t_c0_w);
    Triple p_c1 = R_w_c1 * (p_w - t_c1_w);

    Eigen::Matrix<double, 4, 3> dz_dpc0 = Eigen::Matrix<double, 4, 3>::Zero();
    dz_dpc0(0, 0) = 1 / p_c0(2);
    dz_dpc0(1, 1) = 1 / p_c0(2);
    dz_dpc0(0, 2) = -p_c0(0) / (p_c0(2) * p_c0(2));
    dz_dpc0(1, 2) = -p_c0(1) / (p_c0(2) * p_c0(2));

    Eigen::Matrix<double, 4, 3> dz_dpc1 = Eigen::Matrix<double, 4, 3>::Zero();
    dz_dpc1(2, 0) = 1 / p_c1(2);
    dz_dpc1(3, 1) = 1 / p_c1(2);
    dz_dpc1(2, 2) = -p_c1(0) / (p_c1(2) * p_c1(2));
    dz_dpc1(3, 2) = -p_c1(1) / (p_c1(2) * p_c1(2));

    Eigen::Matrix<double, 3, 6> dpc0_dxc = Eigen::Matrix<double, 3, 6>::Zero();
    Eigen::Matrix<double, 3, 6> dpc1_dxc = Eigen::Matrix<double, 3, 6>::Zero();

    if (clone == CAMERA) {
        if (vfusetype == "Normal") {
            //dpc0_dxc.leftCols(3) = skew(p_c0);
            //dpc0_dxc.rightCols(3) = -R_w_c0;
            dpc0_dxc.leftCols(3) = R_w_c0 * skew(p_w - t_c0_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            //dpc1_dxc.leftCols(3) = R_c0_c1 * skew(p_c0);
            //dpc1_dxc.rightCols(3) = -R_w_c1;
            dpc1_dxc.leftCols(3) = R_w_c1 * skew(p_w - t_c0_w);
            dpc1_dxc.rightCols(3) = R_w_c1;

            //r = Eigen::Vector4d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2),
            //    p_c1(0) / p_c1(2), p_c1(1) / p_c1(2)) - z;
            r = z - Eigen::Vector4d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2),
                p_c1(0) / p_c1(2), p_c1(1) / p_c1(2));
        }
        else if (vfusetype == "InEKF" || vfusetype == "I_InEKF") {
            dpc0_dxc.leftCols(3) = -R_w_c0 * skew(p_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            dpc1_dxc.leftCols(3) = -R_w_c1 * skew(p_w + R_w_c1.transpose() * T_cam0_cam1.translation());
            dpc1_dxc.rightCols(3) = R_w_c1;

            r = z - Eigen::Vector4d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2),
                p_c1(0) / p_c1(2), p_c1(1) / p_c1(2));
        }
    }
    else if (clone == IMU) {
        if (vfusetype == "Normal") {

            dpc0_dxc.leftCols(3) = R_w_c0 * skew(p_w - t_i_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            dpc1_dxc.leftCols(3) = R_w_c1 * skew(p_w - t_i_w);
            dpc1_dxc.rightCols(3) = R_w_c1;

            r = z - Eigen::Vector4d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2),
                p_c1(0) / p_c1(2), p_c1(1) / p_c1(2));
        }
        else if (vfusetype == "InEKF" || vfusetype == "I_InEKF") {
            dpc0_dxc.leftCols(3) = -R_w_c0 * skew(p_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            dpc1_dxc.leftCols(3) = -R_w_c1 * skew(p_w + R_w_c1.transpose() * T_cam0_cam1.translation());
            dpc1_dxc.rightCols(3) = R_w_c1;

            r = z - Eigen::Vector4d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2),
                p_c1(0) / p_c1(2), p_c1(1) / p_c1(2));
        }
    
    }
    

    SO3 dpc0_dpg = R_w_c0;
    SO3 dpc1_dpg = R_w_c1;

    H_x = dz_dpc0 * dpc0_dxc + dz_dpc1 * dpc1_dxc;
    H_f = dz_dpc0 * dpc0_dpg + dz_dpc1 * dpc1_dpg;

    //if (vfusetype == "Normal") {
    //    //OC
    //    Eigen::Matrix<double, 4, 6> A = H_x;
    //    Eigen::Matrix<double, 6, 1> u = Eigen::Matrix<double, 6, 1>::Zero();
    //    u.block<3, 1>(0, 0) = cam_state.orientation_null.toRotationMatrix() * cam_state.gravity;
    //    u.block<3, 1>(3, 0) = skew(p_w - cam_state.position_null) * cam_state.gravity;
    //    H_x = A - A * u * (u.transpose() * u).inverse() * u.transpose();
    //    H_f = -H_x.block<4, 3>(0, 3);
    //}

}


void hwa_vis::vis_base::measurementJacobianEX(
    const CamStateIDType& cam_state_id,
    const FeatureIDType& feature_id,
    Eigen::Matrix<double, 2, 6>& H_x, Eigen::Matrix<double, 2, 6>& H_x_ex, Eigen::Matrix<double, 2, 3>& H_f, Eigen::Vector2d& r)
{


    const CamState& imu_state = imu_states[cam_state_id];
    const Triple& t_i_w = imu_state.position;
    SO3 R_c0_i = T_cam0_imu.linear();
    SO3 R_i_c0 = R_c0_i.transpose();

    const CamState& cam_state = cam_states[cam_state_id];
    const vis_feature& feature = map_server[feature_id];

    //SO3 R_w_c0 = cam_state.orientation.toRotationMatrix();
    SO3 R_w_c0 = cam_state.orientation.toRotationMatrix().transpose();
    const Triple& t_c0_w = cam_state.position;

    const Triple& p_w = feature.position;
    const Eigen::Vector2d z = Eigen::Vector2d(feature.observations.find(cam_state_id)->second(0),
        feature.observations.find(cam_state_id)->second(1));

    Triple p_c0 = R_w_c0 * (p_w - t_c0_w);


    Eigen::Matrix<double, 2, 3> dz_dpc0 = Eigen::Matrix<double, 2, 3>::Zero();
    dz_dpc0(0, 0) = 1 / p_c0(2);
    dz_dpc0(1, 1) = 1 / p_c0(2);
    dz_dpc0(0, 2) = -p_c0(0) / (p_c0(2) * p_c0(2));
    dz_dpc0(1, 2) = -p_c0(1) / (p_c0(2) * p_c0(2));

    Eigen::Matrix<double, 3, 6> dpc0_dxc_ex = Eigen::Matrix<double, 3, 6>::Zero();
    Eigen::Matrix<double, 3, 6> dpc0_dxc = Eigen::Matrix<double, 3, 6>::Zero();

    if (clone == CAMERA) {
        if (vfusetype == "Normal") {
            dpc0_dxc.leftCols(3) = R_w_c0 * skew(p_w - t_c0_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            r = z - Eigen::Vector2d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2));
        }
        else if (vfusetype == "InEKF" || vfusetype == "I_InEKF") {
            dpc0_dxc.leftCols(3) = -R_w_c0 * skew(p_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            r = z - Eigen::Vector2d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2));
        }
    }
    else if (clone == IMU) {
        if (vfusetype == "Normal") {

            dpc0_dxc.leftCols(3) = R_w_c0 * skew(p_w - t_i_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            dpc0_dxc_ex.leftCols(3) = -skew(R_w_c0 * (p_w - t_c0_w));
            dpc0_dxc_ex.rightCols(3) = -R_i_c0;

            r = z - Eigen::Vector2d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2));
        }
        else if (vfusetype == "InEKF" || vfusetype == "I_InEKF") {
            dpc0_dxc.leftCols(3) = -R_w_c0 * skew(p_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            dpc0_dxc_ex.leftCols(3) = -skew(R_w_c0 * (p_w - t_c0_w));
            dpc0_dxc_ex.rightCols(3) = -R_i_c0;

            r = z - Eigen::Vector2d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2));
        }

    }

    SO3 dpc0_dpg = R_w_c0;

    H_x = dz_dpc0 * dpc0_dxc;
    H_f = dz_dpc0 * dpc0_dpg;

    //if (vfusetype == "Normal") {
    //    //OC
    //    Eigen::Matrix<double, 2, 6> A = H_x;
    //    Eigen::Matrix<double, 6, 1> u = Eigen::Matrix<double, 6, 1>::Zero();
    //    u.block<3, 1>(0, 0) = cam_state.orientation_null.toRotationMatrix() * cam_state.gravity;
    //    u.block<3, 1>(3, 0) = skew(p_w - cam_state.position_null) * cam_state.gravity;
    //    H_x = A - A * u * (u.transpose() * u).inverse() * u.transpose();
    //    H_f = -H_x.block<2, 3>(0, 3);
    //}
}

void hwa_vis::vis_base::measurementJacobian(
    const CamStateIDType& cam_state_id,
    const FeatureIDType& feature_id,
    Eigen::Matrix<double, 2, 6>& H_x, Eigen::Matrix<double, 2, 3>& H_f, Eigen::Vector2d& r)
{
    const CamState& imu_state = imu_states[cam_state_id];
    const Triple& t_i_w = imu_state.position;

    const CamState& cam_state = cam_states[cam_state_id];
    const vis_feature& feature = map_server[feature_id];

    //SO3 R_w_c0 = cam_state.orientation.toRotationMatrix();
    SO3 R_w_c0 = cam_state.orientation.toRotationMatrix().transpose();
    const Triple& t_c0_w = cam_state.position;

    const Triple& p_w = feature.position;
    const Eigen::Vector2d z = Eigen::Vector2d(feature.observations.find(cam_state_id)->second(0),
        feature.observations.find(cam_state_id)->second(1));

    Triple p_c0 = R_w_c0 * (p_w - t_c0_w);


    Eigen::Matrix<double, 2, 3> dz_dpc0 = Eigen::Matrix<double, 2, 3>::Zero();
    dz_dpc0(0, 0) = 1 / p_c0(2);
    dz_dpc0(1, 1) = 1 / p_c0(2);
    dz_dpc0(0, 2) = - p_c0(0) / (p_c0(2) * p_c0(2));
    dz_dpc0(1, 2) = - p_c0(1) / (p_c0(2) * p_c0(2));

    Eigen::Matrix<double, 3, 6> dpc0_dxc = Eigen::Matrix<double, 3, 6>::Zero();

    if (clone == CAMERA) {
        if (vfusetype == "Normal") {
            dpc0_dxc.leftCols(3) = R_w_c0 * skew(p_w - t_c0_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            r = z - Eigen::Vector2d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2));
        }
        else if (vfusetype == "InEKF" || vfusetype == "I_InEKF") {
            dpc0_dxc.leftCols(3) = -R_w_c0 * skew(p_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            r = z - Eigen::Vector2d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2));
        }
    }
    else if (clone == IMU) {
        if (vfusetype == "Normal") {

            dpc0_dxc.leftCols(3) = R_w_c0 * skew(p_w - t_i_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            r = z - Eigen::Vector2d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2));
        }
        else if (vfusetype == "InEKF" || vfusetype == "I_InEKF") {
            dpc0_dxc.leftCols(3) = -R_w_c0 * skew(p_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            r = z - Eigen::Vector2d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2));
        }

    }

    SO3 dpc0_dpg = R_w_c0;

    H_x = dz_dpc0 * dpc0_dxc;
    H_f = dz_dpc0 * dpc0_dpg;

    //if (vfusetype == "Normal") {
    //    //OC
    //    Eigen::Matrix<double, 2, 6> A = H_x;
    //    Eigen::Matrix<double, 6, 1> u = Eigen::Matrix<double, 6, 1>::Zero();
    //    u.block<3, 1>(0, 0) = cam_state.orientation_null.toRotationMatrix() * cam_state.gravity;
    //    u.block<3, 1>(3, 0) = skew(p_w - cam_state.position_null) * cam_state.gravity;
    //    H_x = A - A * u * (u.transpose() * u).inverse() * u.transpose();
    //    H_f = -H_x.block<2, 3>(0, 3);
    //}
}


bool hwa_vis::vis_base::featureJacobian(const FeatureIDType& feature_id,
    const std::vector<CamStateIDType>& cam_state_ids,
    Matrix& H_x, Vector& r)
{

    const auto& feature = map_server[feature_id];


    std::vector<CamStateIDType> valid_cam_state_ids(0);
    for (const auto& cam_id : cam_state_ids)
    {
        if (feature.observations.find(cam_id) == feature.observations.end())
            continue;

        valid_cam_state_ids.push_back(cam_id);
    }

    int jacobian_row_size = 0;

    if (stereo) jacobian_row_size = 4 * valid_cam_state_ids.size();
    else jacobian_row_size = 2 * valid_cam_state_ids.size();

    //Matrix H_xj = Matrix::Zero(jacobian_row_size,
    //    15 + cam_states.size() * 6);

    Matrix H_xj;
    if (imgproc->estimate_extrinsic)
        H_xj = Matrix::Zero(jacobian_row_size, cam_states.size() * 6 + 6);
    else
        H_xj = Matrix::Zero(jacobian_row_size, cam_states.size() * 6);
    Matrix H_fj = Matrix::Zero(jacobian_row_size, 3);
    Vector r_j = Vector::Zero(jacobian_row_size);
    int stack_cntr = 0;
    if (stereo)
    {
        for (const auto& cam_id : valid_cam_state_ids)
        {
            Eigen::Matrix<double, 4, 6> H_xi = Eigen::Matrix<double, 4, 6>::Zero();
            Eigen::Matrix<double, 4, 6> H_xi_ex = Eigen::Matrix<double, 4, 6>::Zero();
            Eigen::Matrix<double, 4, 3> H_fi = Eigen::Matrix<double, 4, 3>::Zero();
            Eigen::Vector4d r_i = Eigen::Vector4d::Zero();

            if(imgproc->estimate_extrinsic) measurementJacobianEX(cam_id, feature.id, H_xi,H_xi_ex,H_fi, r_i);
            else measurementJacobian(cam_id, feature.id, H_xi, H_fi, r_i);

            auto cam_state_iter = cam_states.find(cam_id);
            int cam_state_cntr;

            if (imgproc->estimate_extrinsic) {
                H_xj.block<4, 6>(stack_cntr, 0) = H_xi_ex;
                cam_state_cntr = std::distance(cam_states.begin(), cam_state_iter) + 1;
            }            
            else
                cam_state_cntr = std::distance(cam_states.begin(), cam_state_iter);


            //H_xj.block<4, 6>(stack_cntr, 15 + 6 * cam_state_cntr) = H_xi;
            H_xj.block<4, 6>(stack_cntr, 6 * cam_state_cntr) = H_xi;
            H_fj.block<4, 3>(stack_cntr, 0) = H_fi;
            r_j.segment<4>(stack_cntr) = r_i;
            stack_cntr += 4;
        }
    }
    else
    {
        for (const auto& cam_id : valid_cam_state_ids)
        {
            Eigen::Matrix<double, 2, 6> H_xi = Eigen::Matrix<double, 2, 6>::Zero();
            Eigen::Matrix<double, 2, 3> H_fi = Eigen::Matrix<double, 2, 3>::Zero();
            Eigen::Matrix<double, 2, 6> H_xi_ex = Eigen::Matrix<double, 2, 6>::Zero();
            Eigen::Vector2d r_i = Eigen::Vector2d::Zero();

            if (imgproc->estimate_extrinsic) measurementJacobianEX(cam_id, feature.id, H_xi, H_xi_ex, H_fi, r_i);
            else measurementJacobian(cam_id, feature.id, H_xi, H_fi, r_i);

            auto cam_state_iter = cam_states.find(cam_id);

            int cam_state_cntr;
            if (imgproc->estimate_extrinsic) {
                H_xj.block<2, 6>(stack_cntr, 0) = H_xi_ex;
                cam_state_cntr = std::distance(cam_states.begin(), cam_state_iter) + 1;
            }
            else
                cam_state_cntr = std::distance(cam_states.begin(), cam_state_iter);


            H_xj.block<2, 6>(stack_cntr, 6 * cam_state_cntr) = H_xi;
            //H_xj.block<2, 6>(stack_cntr, 15 + 6 * cam_state_cntr) = H_xi;
            H_fj.block<2, 3>(stack_cntr, 0) = H_fi;
            r_j.segment<2>(stack_cntr) = r_i;
            stack_cntr += 2;
        }
    }

    // more test need to do
    //residual check //
    /*double residual_mean = r_j.mean();
    double residual_std = 0.0;
    std::cout << "r:" << r_j.transpose() << std::endl;
    for (size_t i = 0; i < r_j.rows(); i++)
    {
        residual_std += (r_j(i) - residual_mean)*(r_j(i) - residual_mean)/ (r_j.rows() - 1);
        std::cout << "fabs(r_j(i)*cam0_intrinsics(0)):" << fabs(r_j(i)*cam0_intrinsics(0)) << std::endl;
        if (fabs(r_j(i)*cam0_intrinsics(0)) > 4.0)
        {
            residual_std = 25.0 / cam0_intrinsics(0) / cam0_intrinsics(0);
            break;
        }
    }
    residual_std = sqrt(residual_std);
    std::cout << "residual_std:" << residual_std << std::endl;
    std::cout << "residual_std*cam0_intrinsics(0):" << residual_std * cam0_intrinsics(0) << std::endl;
    if (residual_std*cam0_intrinsics(0) > 2.0)
    {
        std::cout << "Residual_big" << std::endl;
        return false;
    }*/
    if (H_fj.rows() == 0) return false;

    Eigen::JacobiSVD<Matrix> svd_helper(H_fj, Eigen::ComputeFullU | Eigen::ComputeThinV);
    Matrix A = svd_helper.matrixU().rightCols(
        jacobian_row_size - 3);
    
    H_x = A.transpose() * H_xj;
    r = A.transpose() * r_j;

    //std::cout << "Test: " <<std::setiosflags(ios::fixed)<<std::setprecision(6)<< std::endl << A.transpose() * H_fj<<std::endl<<std::endl;

   /* std::cout << "Before The Zero Projection -- H" << std::endl << std::setiosflags(ios::fixed) << std::setprecision(2) << H_xj << std::endl << std::endl;
    std::cout << "After The Zero Projection -- H" << std::endl << std::setiosflags(ios::fixed) << std::setprecision(2) << H_x << std::endl << std::endl;
    std::cout << "Before The Zero Projection -- R" << std::endl << std::setiosflags(ios::fixed) << std::setprecision(3) << r_j.transpose() << std::endl << std::endl;
    std::cout << "After The Zero Projection -- R" << std::endl << std::setiosflags(ios::fixed) << std::setprecision(3) << r.transpose() << std::endl << std::endl;*/

    return true;
}

void hwa_vis::vis_base::findRedundantCamStates(std::vector<CamStateIDType>& rm_cam_state_ids)
{
    if (!stereo)
    {
        auto first_cam_state_iter = KeyFrameBuffer.begin();
        rm_cam_state_ids.emplace_back(*first_cam_state_iter);
        ++first_cam_state_iter;
        rm_cam_state_ids.emplace_back(*first_cam_state_iter);
    }
    else
    {
        auto key_cam_state_iter = KeyFrameBuffer.end();

        for (int i = 0; i < 4; ++i)
            --key_cam_state_iter;

        auto cam_state_iter = key_cam_state_iter;
        ++cam_state_iter;
        auto first_cam_state_iter = KeyFrameBuffer.begin();
        const Triple key_position =
            cam_states[*key_cam_state_iter].position;
        const SO3 key_rotation =
            cam_states[*key_cam_state_iter].orientation.toRotationMatrix();

        for (int i = 0; i < 2; ++i)
        {
            const Triple position =
                cam_states[*cam_state_iter].position;
            const SO3 rotation =
                cam_states[*cam_state_iter].orientation.toRotationMatrix();

            double distance = (position - key_position).norm();
            double angle = Eigen::AngleAxisd(
                rotation * key_rotation.transpose()).angle();

            if (angle < 0.2618 && distance < 0.5 && cam_states[*cam_state_iter].mtracking_rate > 0.5)  // msckf used para
            {
                rm_cam_state_ids.push_back(*cam_state_iter);
                ++cam_state_iter;
            }
            else
            {
                rm_cam_state_ids.push_back(*first_cam_state_iter);
                ++first_cam_state_iter;

            }
        }
    }

    std::sort(rm_cam_state_ids.begin(), rm_cam_state_ids.end());

    std::vector<CamStateIDType> vTemp;
    bool isRemove;
    for (int i = 0; i < KeyFrameBuffer.size(); i++)
    {
        isRemove = false;

        for (int j = 0; j < rm_cam_state_ids.size(); j++)
        {
            if (KeyFrameBuffer[i] == rm_cam_state_ids[j])
                isRemove = true;
        }

        if (!isRemove)
            vTemp.push_back(KeyFrameBuffer[i]);
    }
    KeyFrameBuffer = vTemp;
    return;
}

void hwa_vis::vis_base::findNonKeyCamStates(std::vector<CamStateIDType>& rm_cam_state_ids)
{
    auto key_cam_state_iter = nonKeyFrameBuffer.end();


    for (int i = 0; i < 4; ++i)
        --key_cam_state_iter;

    auto cam_state_iter = key_cam_state_iter;
    ++cam_state_iter;

    auto first_cam_state_iter = nonKeyFrameBuffer.begin();

    const Triple key_position =
        cam_states[*key_cam_state_iter].position;


    const SO3 key_rotation =
        cam_states[*key_cam_state_iter].orientation.toRotationMatrix().transpose();

    for (int i = 0; i < 2; ++i)
    {
        const Triple position =
            cam_states[*cam_state_iter].position;
        const SO3 rotation =
            cam_states[*cam_state_iter].orientation.toRotationMatrix().transpose();

        double distance = (position - key_position).norm();
        double angle = Eigen::AngleAxisd(
            rotation * key_rotation.transpose()).angle();

        if (angle < 0.01 && distance < 0.7
            && cam_states[*cam_state_iter].mtracking_rate > 0.5)
        {
            rm_cam_state_ids.push_back(*cam_state_iter);
            ++cam_state_iter;
        }
        else
        {
            rm_cam_state_ids.push_back(*first_cam_state_iter);
            ++first_cam_state_iter;
        }
    }

    sort(rm_cam_state_ids.begin(), rm_cam_state_ids.end());

    std::vector<CamStateIDType> vTemp;

    for (int i = 0; i < nonKeyFrameBuffer.size(); i++)
    {
        if ((nonKeyFrameBuffer[i] != rm_cam_state_ids[0])
            && (nonKeyFrameBuffer[i] != rm_cam_state_ids[1]))
            vTemp.push_back(nonKeyFrameBuffer[i]);
    }
    nonKeyFrameBuffer = vTemp;
    return;
}

void hwa_vis::vis_base::UpdateVisualFeautures()
{

    visual_features.clear();
    for (auto iter : map_server)
    {
        if (iter.second.is_initialized)
        {
            visual_features.push_back(iter.second.position);
        }
    }

}

// R_e_c
void hwa_vis::GlobalSFM::triangulatePoint(Eigen::Matrix<double, 3, 4>& Pose0, Eigen::Matrix<double, 3, 4>& Pose1,
    Eigen::Vector2d& point0, Eigen::Vector2d& point1, Triple& point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
        design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);

}

// Need some correction
bool hwa_vis::GlobalSFM::solveFrameByPnP(SO3& R_initial, Triple& P_initial, int i,
    std::vector<SFMFeature>& sfm_f)
{
    std::vector<cv::Point2f> pts_2_vector;
    std::vector<cv::Point3f> pts_3_vector;
    for (int j = 0; j < feature_num; j++)
    {
        if (sfm_f[j].state != true)
            continue;
        Eigen::Vector2d point2d;
        for (int k = 0; k < (int)sfm_f[j].observation.size(); k++)
        {
            if (sfm_f[j].observation[k].first == i)    // �Ѿ��ɹ��ָ���άλ�õĵ�
            {
                Eigen::Vector2d img_pts = sfm_f[j].observation[k].second;
                cv::Point2f pts_2(img_pts(0), img_pts(1));
                pts_2_vector.push_back(pts_2);
                cv::Point3f pts_3(sfm_f[j].position[0], sfm_f[j].position[1], sfm_f[j].position[2]);
                pts_3_vector.push_back(pts_3);
                break;
            }
        }
    }
    if (int(pts_2_vector.size()) < 15)
    {
        std::cout<<"unstable features tracking, please slowly move you device!"<< std::endl;
        if (int(pts_2_vector.size()) < 10)
            return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1);
    if (!pnp_succ)
    {
        return false;
    }
    cv::Rodrigues(rvec, r);
    //std::cout << "r " << std::endl << r << std::endl;
    Matrix R_pnp;
    cv::cv2eigen(r, R_pnp);
    Matrix T_pnp;
    cv::cv2eigen(t, T_pnp);
    R_initial = R_pnp;
    P_initial = T_pnp;
    return true;

}

void hwa_vis::GlobalSFM::triangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4>& Pose0,
    int frame1, Eigen::Matrix<double, 3, 4>& Pose1,
    std::vector<SFMFeature>& sfm_f)
{
    assert(frame0 != frame1);
    for (int j = 0; j < feature_num; j++)
    {
        if (sfm_f[j].state == true)
            continue;
        bool has_0 = false, has_1 = false;
        Eigen::Vector2d point0;
        Eigen::Vector2d point1;
        for (int k = 0; k < (int)sfm_f[j].observation.size(); k++)
        {
            if (sfm_f[j].observation[k].first == frame0)
            {
                point0 = sfm_f[j].observation[k].second;
                has_0 = true;
            }
            if (sfm_f[j].observation[k].first == frame1)
            {
                point1 = sfm_f[j].observation[k].second;
                has_1 = true;
            }
        }
        if (has_0 && has_1)
        {
            Triple point_3d;
            triangulatePoint(Pose0, Pose1, point0, point1, point_3d);
            sfm_f[j].state = true;
            sfm_f[j].position[0] = point_3d(0);
            sfm_f[j].position[1] = point_3d(1);
            sfm_f[j].position[2] = point_3d(2);
        }
    }
}
// 	 q w_R_cam t w_R_cam
//  c_rotation cam_R_w 
//  c_translation cam_R_w
// relative_q[i][j]  j_q_i
// relative_t[i][j]  j_t_ji  (j < i)
bool hwa_vis::GlobalSFM::construct(int frame_num, std::vector<Eigen::Quaterniond>& q, std::vector<Triple>& T, int& l,
    const SO3 relative_R, const Triple relative_T,
    std::vector<SFMFeature>& sfm_f, std::map<int, Triple>& sfm_tracked_points)
{
    int count = frame_num - 1;
    feature_num = sfm_f.size();

    q[l].w() = 1;
    q[l].x() = 0;
    q[l].y() = 0;
    q[l].z() = 0;
    T[l].setZero();

    q[count] = q[l] * Eigen::Quaterniond(relative_R);   //R_n_e   
    T[count] = relative_T;  //T_n_l

    SO3* c_Rotation = new SO3[frame_num];
    Triple* c_Translation = new Triple[frame_num];
    Eigen::Quaterniond* c_Quat = new Eigen::Quaterniond[frame_num];
    Eigen::Matrix<double, 3, 4>* Pose = new Eigen::Matrix<double, 3, 4>[frame_num];
    double(*c_rotation)[4] = new double[frame_num][4];
    double(*c_translation)[3] = new double[frame_num][3];

    c_Quat[l] = q[l].inverse();   //R_e_c
    c_Rotation[l] = c_Quat[l].toRotationMatrix();
    c_Translation[l] = - c_Rotation[l] * T[l];
    Pose[l].block<3, 3>(0, 0) = c_Rotation[l];
    Pose[l].block<3, 1>(0, 3) = c_Translation[l];

    c_Quat[count] = q[count].inverse();  //R_e_n
    c_Rotation[count] = c_Quat[count].toRotationMatrix();
    c_Translation[count] = - c_Rotation[count] * T[count];
    Pose[count].block<3, 3>(0, 0) = c_Rotation[count];
    Pose[count].block<3, 1>(0, 3) = c_Translation[count];

    for (int i = l; i < count; i++)
    {
        if (i > l) 
        {
            SO3 R_initial = c_Rotation[i - 1];  
            Triple P_initial = c_Translation[i - 1];
            if (!solveFrameByPnP(R_initial, P_initial, i, sfm_f)){
                delete[] c_Rotation;
                delete[] c_Translation;
                delete[] c_Quat;
                delete[] Pose;
                delete[] c_rotation;
                delete[] c_translation;
                return false;
            }
            c_Rotation[i] = R_initial;
            c_Translation[i] = P_initial;
            c_Quat[i] = c_Rotation[i];
            Pose[i].block<3, 3>(0, 0) = c_Rotation[i];     //R_e_i
            Pose[i].block<3, 1>(0, 3) = c_Translation[i];
        }

        triangulateTwoFrames(i, Pose[i], count, Pose[count], sfm_f);
    }

    for (int i = l + 1; i < frame_num - 1; i++)
        triangulateTwoFrames(l, Pose[l], i, Pose[i], sfm_f);

    for (int i = l - 1; i >= 0; i--)
    {
        SO3 R_initial = c_Rotation[i + 1];
        Triple P_initial = c_Translation[i + 1];
        if (!solveFrameByPnP(R_initial, P_initial, i, sfm_f)) {
            delete[] c_Rotation;
            delete[] c_Translation;
            delete[] c_Quat;
            delete[] Pose;
            delete[] c_rotation;
            delete[] c_translation;
            return false;
        }

        c_Rotation[i] = R_initial;
        c_Translation[i] = P_initial;
        c_Quat[i] = c_Rotation[i];
        Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
        Pose[i].block<3, 1>(0, 3) = c_Translation[i];

        triangulateTwoFrames(i, Pose[i], l, Pose[l], sfm_f);
    }

    for (int j = 0; j < feature_num; j++)
    {
        if (sfm_f[j].state == true)
            continue;
        if ((int)sfm_f[j].observation.size() >= 2)
        {
            Eigen::Vector2d point0, point1;
            int frame_0 = sfm_f[j].observation[0].first;
            point0 = sfm_f[j].observation[0].second;
            int frame_1 = sfm_f[j].observation.back().first;
            point1 = sfm_f[j].observation.back().second;
            Triple point_3d;
            triangulatePoint(Pose[frame_0], Pose[frame_1], point0, point1, point_3d);
            sfm_f[j].state = true;
            sfm_f[j].position[0] = point_3d(0);
            sfm_f[j].position[1] = point_3d(1);
            sfm_f[j].position[2] = point_3d(2);
        }
    }

    ceres::Problem problem;
    ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
    for (int i = 0; i < frame_num; i++)
    {
        c_translation[i][0] = c_Translation[i].x();
        c_translation[i][1] = c_Translation[i].y();
        c_translation[i][2] = c_Translation[i].z();
        c_rotation[i][0] = c_Quat[i].w();
        c_rotation[i][1] = c_Quat[i].x();
        c_rotation[i][2] = c_Quat[i].y();
        c_rotation[i][3] = c_Quat[i].z();
        problem.AddParameterBlock(c_rotation[i], 4, local_parameterization);
        problem.AddParameterBlock(c_translation[i], 3);
        if (i == l || i == frame_num - 1)
        {
            problem.SetParameterBlockConstant(c_rotation[i]);
            problem.SetParameterBlockConstant(c_translation[i]);
        }
    }

    for (int i = 0; i < feature_num; i++)
    {
        if (sfm_f[i].state != true)
            continue;
        for (int j = 0; j < int(sfm_f[i].observation.size()); j++)
        {
            int l = sfm_f[i].observation[j].first;
            ceres::CostFunction* cost_function = ReprojectionError3D::Create(
                sfm_f[i].observation[j].second.x(),
                sfm_f[i].observation[j].second.y());
            problem.AddResidualBlock(cost_function, NULL, c_rotation[l], c_translation[l],
                sfm_f[i].position);
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_solver_time_in_seconds = 0.2;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (summary.termination_type != ceres::CONVERGENCE && summary.final_cost >= 5e-03) {
        delete[] c_Rotation;
        delete[] c_Translation;
        delete[] c_Quat;
        delete[] Pose;
        delete[] c_rotation;
        delete[] c_translation;
        return false;
    }

    for (int i = 0; i < frame_num; i++)
    {
        q[i].w() = c_rotation[i][0];
        q[i].x() = c_rotation[i][1];
        q[i].y() = c_rotation[i][2];
        q[i].z() = c_rotation[i][3];
        q[i] = q[i].inverse(); //R_c_e 
    }

    for (int i = 0; i < frame_num; i++)
    {
        T[i] = -1 * (q[i] * Triple(c_translation[i][0], c_translation[i][1], c_translation[i][2]));
        //std::cout << "final  t" << " i " << i <<"  " << T[i](0) <<"  "<< T[i](1) <<"  "<< T[i](2) << std::endl;
    }

    for (int i = 0; i < (int)sfm_f.size(); i++)
    {
        if (sfm_f[i].state)
            sfm_tracked_points[sfm_f[i].id] = Triple(sfm_f[i].position[0], sfm_f[i].position[1], sfm_f[i].position[2]);
    }

    delete[] c_Rotation;
    delete[] c_Translation;
    delete[] c_Quat;
    delete[] Pose;
    delete[] c_rotation;
    delete[] c_translation;

    return true;
}

//Vins
bool hwa_vis::vis_base::relativePose(SO3& relative_R, Triple& relative_T, int& l)
{
    for (int i = 0; i < frame_count - 1; i++)
    {
        std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> corres;
        corres = getCorresponding(i, frame_count - 1);

        if (corres.size() > 20)
        {
            double sum_parallax = 0;  
            double average_parallax;  
            for (int j = 0; j < int(corres.size()); j++)
            {
                Eigen::Vector2d pts_0(corres[j].first(0), corres[j].first(1));  
                Eigen::Vector2d pts_1(corres[j].second(0), corres[j].second(1)); 
                double parallax = (pts_0 - pts_1).norm();  
                sum_parallax = sum_parallax + parallax;  
            }

            average_parallax = 1.0 * sum_parallax / int(corres.size());

            if (average_parallax * 460 < 30) {
                //std::cout << "Not Enough parallax" << std::endl;
                return false;
            }
            if (average_parallax * 460 > 30 && solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;  
                std::cout << "average_parallax: "<< average_parallax * 460<< " Newest frame to triangulate the whole structure: "<< l << std::endl ;
                return true; 
            }
        }
    }
    return false; 
}

std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> hwa_vis::vis_base::getCorresponding(int frame_count_l, int frame_count_r)
{
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> corres;

    for (auto& It : map_server)
    {
        hwa_vis::vis_feature it = It.second;
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Eigen::Vector2d a = Eigen::Vector2d::Zero(), b = Eigen::Vector2d::Zero(); 
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;
            auto tmp_it = it.observations.begin();
            std::advance(tmp_it, idx_l);  
            a = tmp_it->second.head<2>(); 
            std::advance(tmp_it, idx_r);  
            b = tmp_it->second.head<2>();
            corres.push_back(std::make_pair(a, b));
        }
    }
    return corres;
}



bool hwa_vis::vis_base::solveRelativeRT(const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& corres, SO3& Rotation, Triple& Translation)
{
    if (corres.size() >= 15)
    {
        std::vector<cv::Point2f> ll, rr;
        for (int i = 0; i < int(corres.size()); i++)
        {
            ll.push_back(cv::Point2f(corres[i].first(0), corres[i].first(1)));  
            rr.push_back(cv::Point2f(corres[i].second(0), corres[i].second(1))); 
        }

        cv::Mat mask;
        cv::Mat E = cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 0.3 / 460, 0.99, mask);
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        cv::Mat rot, trans;
        int inlier_cnt = cv::recoverPose(E, ll, rr, cameraMatrix, rot, trans, mask);
        SO3 R;
        Triple T;
        for (int i = 0; i < 3; i++)
        {
            T(i) = trans.at<double>(i, 0); 
            for (int j = 0; j < 3; j++)
                R(i, j) = rot.at<double>(i, j);  
        }

        Rotation = R.transpose(); 
        Translation = -R.transpose() * T; 
        if (inlier_cnt > 12)
            return true;  
        else
            return false; 
    }
    return false; 
}

bool hwa_vis::vis_base::addFeatureObservations()
{
    int curr_feature_num = map_server.size();
    int tracked_feature_num = 0;

    for (const auto& feature : _pointcloud.features)
    {
        if (map_server.find(feature.id) == map_server.end())
        {
            hwa_vis::FeatureIDType feature_id = feature.id;
            map_server[feature.id] = hwa_vis::vis_feature(feature.id);
            if (stereo)
            {
                map_server[feature.id].observations[cam_state_id] =
                    Eigen::Vector4d(feature.cam0_point.x, feature.cam0_point.y,
                        feature.cam1_point.x, feature.cam1_point.y);
            }
            else
            {
                map_server[feature.id].observations[cam_state_id] =
                    Eigen::Vector4d(feature.cam0_point.x, feature.cam0_point.y,
                        0.0, 0.0);
            }

            if (usingstereorecity && stereo)
                map_server[feature.id].initial_depth = feature.depth;

            if (isKeyFrame)
                map_server[feature.id].is_KeyFrame = true;
        }
        else
        {
            if (stereo)
            {
                map_server[feature.id].observations[cam_state_id] =
                    Eigen::Vector4d(feature.cam0_point.x, feature.cam0_point.y,
                        feature.cam1_point.x, feature.cam1_point.y);
            }
            else
            {
                map_server[feature.id].observations[cam_state_id] =
                    Eigen::Vector4d(feature.cam0_point.x, feature.cam0_point.y,
                        0.0, 0.0);
            }
            ++tracked_feature_num;
        }
    }

    for (auto iter = map_server.begin();
        iter != map_server.end(); ++iter)
    {
        auto& feature = iter->second;
        if (feature.observations.find(cam_state_id) ==
            feature.observations.end())
        {
            feature.isLost = true;
        }

        feature.start_frame_id = feature.observations.begin()->first;
        feature.frame_size = feature.observations.size();
        feature.start_frame = static_cast<int>(std::distance(cam_states.begin(), cam_states.find(feature.start_frame_id)));
    }

    tracking_rate =
        static_cast<double>(tracked_feature_num) /
        static_cast<double>(curr_feature_num);
    std::cout << "tracking_rate:" << tracking_rate << std::endl;
    cam_states[cam_state_id].mtracking_rate = tracking_rate;
    return true;
}

bool hwa_vis::vis_base::checkStaticMotion()
{
    double fx = cam0_intrinsics(0); double fy = cam0_intrinsics(1);
    double cx = cam0_intrinsics(2); double cy = cam0_intrinsics(3);

    double mean_motion = 0.0;
    int common_feature_size = 0;
    for (auto pts : _pointcloud.features)
    {
        int feature_id = pts.id;
        auto iter = map_server.find(feature_id);

        if (iter != map_server.end())
        {
            double x1, y1, x2, y2;
            x1 = fx * pts.cam0_point.x + cx;
            y1 = fy * pts.cam0_point.y + cy;
            x2 = fx * iter->second.observations.rbegin()->second(0) + cx;
            y2 = fy * iter->second.observations.rbegin()->second(1) + cy;
            double dx = x2 - x1;
            double dy = y2 - y1;
            mean_motion += sqrt(dx * dx + dy * dy);
            common_feature_size++;
        }
    }
    mean_motion = mean_motion / common_feature_size;

    if (mean_motion < static_threshold)
    {
        return true;
    }
    return false;
}


hwa_vis::CamStateServer hwa_vis::imuStateServer2CamStateServer(const hwa_vis::CamStateServer& imu_states, const SO3 R_c_i, const Triple t_c_i)
{
    hwa_vis::CamStateServer cam_states = imu_states;
    auto cam_state_iter = cam_states.begin();
    for (int i = 0; i < cam_states.size();
        ++i, ++cam_state_iter)
    {
        cam_state_iter->second.position = cam_state_iter->second.position + cam_state_iter->second.orientation * t_c_i;
        cam_state_iter->second.orientation = cam_state_iter->second.orientation * Eigen::Quaterniond(R_c_i);
    }
    return cam_states;
}

bool hwa_vis::vis_base::initialStructure()
{
    {
        hwa_vis::CamStateServer::iterator frame_it;
        Triple sum_g = Triple::Zero();

        for (frame_it = cam_states.begin(), frame_it++; frame_it != cam_states.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Triple tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }

        Triple aver_g = sum_g * 1.0 / ((int)cam_states.size() - 1);
        double var = 0;

        for (frame_it = cam_states.begin(), frame_it++; frame_it != cam_states.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Triple tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
        }

        var = sqrt(var / ((int)cam_states.size() - 1));
        if (var < 0.25)
        {
            return false;
        }
    }

    std::vector<Eigen::Quaterniond> Q(frame_count, Eigen::Quaterniond::Identity());
    std::vector<Triple> T(frame_count, Triple::Zero());
    std::map<int, Triple> sfm_tracked_points;
    std::vector<hwa_vis::SFMFeature> sfm_f;

    for (auto& it_per_id : map_server)
    {
        auto it = it_per_id.second;
        int imu_j = it.start_frame - 1;
        hwa_vis::SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it.id;

        for (auto& it_per_frame : it.observations)
        {
            imu_j++;
            tmp_feature.observation.push_back(std::make_pair(imu_j, it_per_frame.second.head<2>()));
        }

        sfm_f.push_back(tmp_feature);
    }

    SO3 relative_R;
    Triple relative_T;
    int l;

    if (!relativePose(relative_R, relative_T, l))
    {
        std::cout << "Not enough features or parallax; Move device around" << std::endl;
        return false;
    }

    refer_id = l;
    hwa_vis::GlobalSFM sfm;
    if (!sfm.construct(frame_count, Q, T, l, relative_R, relative_T, sfm_f, sfm_tracked_points))
    {
        std::cout << "global SFM failed!" << std::endl;
        return false;
    }

    hwa_vis::CamStateServer::iterator frame_it;
    std::map<int, Triple>::iterator it;
    frame_it = cam_states.begin();
    hwa_vis::CamStateServer::iterator frame_l = cam_states.begin();
    std::advance(frame_l, l);
    auto frame_b = cam_states.begin();

    for (int i = 0; frame_it != cam_states.end(); frame_it++, i++)
    {
        frame_it->second.orientation = frame_l->second.orientation * Q[i];  //R_Ci_Cl -> R_Ci_W
        frame_it->second.position = frame_b->second.position + frame_l->second.orientation * (T[i] - T[0]);  //T_C_W  
        frame_it->second.TCI();
        std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(10.4) << "Quat:  " << Q[i].x() << "  " << Q[i].y() << "  " << Q[i].z() << "  " << Q[i].w() << std::endl;
    }

    for (int i = 0; i < cam_states.size(); i++)
    {
        std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(10.4) << "Trans: " << T[i].transpose() << std::endl;
    }

    if (visualInitialAlign())
        return true;
    else
    {
        std::cout << "misalign visual structure with IMU" << std::endl;
        return false;
    }
}

bool hwa_vis::vis_base::visualInitialAlign()
{
    Vector x;
    Triple g;
    bool result = VisualIMUAlignment(cam_states, g, x);
    if (!result)
    {
        std::cout << "solve g failed!" << std::endl;
        return false;
    }

    double s = (x.tail<1>())(0);
    int i = 0;
    auto frame_b = cam_states.begin();

    hwa_vis::CamStateServer::iterator frame_l = cam_states.begin();
    std::advance(frame_l, refer_id);

    g = frame_l->second.orientation.conjugate() * g;
    SO3 R0 = hwa_vis::vis_base::g2R(g);
    R0 = hwa_vis::vis_base::ypr2R(Triple{ hwa_vis::vis_base::R2ypr(frame_l->second.qnc.toRotationMatrix()).x(), 0, 0 }) * R0;
    Eigen::Quaterniond orientation_store = frame_l->second.orientation;
    frame_l->second.orientation = frame_l->second.R_e_n.transpose() * R0;

    Eigen::Quaterniond delta_q = frame_l->second.orientation * orientation_store.conjugate();
    g = R0 * g; //ENU
    std::cout << "Gravity N" << g.transpose() << std::endl;
    SO3 rot_diff = delta_q.toRotationMatrix();

    for (auto it = cam_states.begin(); it != cam_states.end(); it++, i++)
    {
        it->second.pre_integration->repropagate(Bas[i], Bgs[i]);
        it->second.orientation = rot_diff * it->second.orientation;
        it->second.position = rot_diff * s * (it->second.position - frame_b->second.position) + frame_b->second.position;
        it->second.TCI();
        it->second.ve = it->second.orientation_b * x.segment<3>(i * 3);
        std::cout << "Velocity " << i << ": " << it->second.ve.transpose() << std::endl;
        it->second.qnb = it->second.R_e_n * it->second.orientation_b;
    }
    return true;
}

bool hwa_vis::vis_base::VisualIMUAlignment(hwa_vis::CamStateServer& camstates, Triple& g, Vector& x)
{
    solveGyroscopeBias(camstates);

    if (LinearAlignment(camstates, g, x))
        return true;
    else
        return false;
}

void hwa_vis::vis_base::solveGyroscopeBias(hwa_vis::CamStateServer& camstates)
{
    SO3 A;
    Triple b;
    Triple delta_bg;
    A.setZero();
    b.setZero();
    hwa_vis::CamStateServer::iterator frame_i;
    hwa_vis::CamStateServer::iterator frame_j;
    for (frame_i = camstates.begin(); next(frame_i) != camstates.end(); frame_i++)
    {
        frame_j = next(frame_i);
        Matrix tmp_A(3, 3);
        tmp_A.setZero();
        Vector tmp_b(3);
        tmp_b.setZero();
        Eigen::Quaterniond q_ij(frame_i->second.orientation_b.conjugate() * frame_j->second.orientation_b);
        // j -> i
        tmp_A = frame_i->second.pre_integration->jacobian.template block<3, 3>(hwa_vis::O_R, hwa_vis::O_BG);
        tmp_b = 2 * (frame_i->second.pre_integration->delta_q.inverse() * q_ij).vec();
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
    }
    delta_bg = A.ldlt().solve(b);
    std::cout << "gyroscope bias initial calibration: " << delta_bg.transpose() << std::endl;
    for (int i = 0; i < frame_count; i++)
        Bgs[i] += delta_bg;

    int i = 0;
    for (frame_i = camstates.begin(); next(frame_i) != camstates.end(); frame_i++, i++)
    {
        frame_i->second.pre_integration->repropagate(Bas[i], Bgs[i]);   // 0 -> i
    }
}

bool hwa_vis::vis_base::LinearAlignment(hwa_vis::CamStateServer& camstates, Triple& g, Vector& x)
{
    int all_frame_count = camstates.size();
    int n_state = all_frame_count * 3 + 3 + 1;


    Matrix A{ n_state, n_state };
    A.setZero();
    Vector b{ n_state };
    b.setZero();

    hwa_vis::CamStateServer::iterator frame_i;
    hwa_vis::CamStateServer::iterator frame_j;
    int i = 0;
    for (frame_i = camstates.begin(); next(frame_i) != camstates.end(); frame_i++, i++)
    {
        frame_j = next(frame_i);

        Matrix tmp_A(6, 10);
        tmp_A.setZero();
        Vector tmp_b(6);
        tmp_b.setZero();

        // j -> i
        double dt = frame_i->second.pre_integration->sum_dt;
        tmp_A.block<3, 3>(0, 0) = -dt * SO3::Identity();
        tmp_A.block<3, 3>(0, 6) = frame_i->second.orientation_b.toRotationMatrix().transpose() * dt * dt / 2 * SO3::Identity();
        tmp_A.block<3, 1>(0, 9) = frame_i->second.orientation_b.toRotationMatrix().transpose() * (frame_j->second.position - frame_i->second.position) / 100.0;
        tmp_b.block<3, 1>(0, 0) = frame_i->second.pre_integration->delta_p + frame_i->second.orientation_b.toRotationMatrix().transpose() * frame_j->second.orientation_b.toRotationMatrix() * frame_j->second.Tbc - frame_j->second.Tbc;
        tmp_A.block<3, 3>(3, 0) = -SO3::Identity();
        tmp_A.block<3, 3>(3, 3) = frame_i->second.orientation_b.toRotationMatrix().transpose() * frame_j->second.orientation_b.toRotationMatrix();
        tmp_A.block<3, 3>(3, 6) = frame_i->second.orientation_b.toRotationMatrix().transpose() * dt * SO3::Identity();
        tmp_b.block<3, 1>(3, 0) = frame_i->second.pre_integration->delta_v;

        Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Zero();
        cov_inv.setIdentity();

        Matrix r_A = tmp_A.transpose() * cov_inv * tmp_A;
        Vector r_b = tmp_A.transpose() * cov_inv * tmp_b;

        A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
        b.segment<6>(i * 3) += r_b.head<6>();

        A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
        b.tail<4>() += r_b.tail<4>();

        A.block<6, 4>(i * 3, n_state - 4) += r_A.topRightCorner<6, 4>();
        A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();
    }
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);
    double s = x(n_state - 1) / 100.0;
    std::cout << "estimated scale: " << s << std::endl;
    g = x.segment<3>(n_state - 4);
    std::cout << " result g:  " << g.norm() << " " << g.transpose() << std::endl;
    if (fabs(g.norm() - 9.8) > 1.0 || s < 0)
    {
        return false;
    }
    Triple Gravity_store = g;
    RefineGravity(camstates, g, x);
    Triple delta_ba = Gravity_store - g;

    std::cout << " Estimated Acc Bias:  " << delta_ba.transpose() << std::endl;

    i = 0;
    //for (auto it = cam_states.begin(); it != cam_states.end(); it++, i++) {
    //    Bas[i] += it->second.orientation_b.conjugate() * delta_ba; // transform �� ��ǰ֡
    //}

    s = (x.tail<1>())(0) / 100.0;
    (x.tail<1>())(0) = s;
    std::cout << " Refined Gravity:  " << g.norm() << " " << g.transpose() << std::endl;

    std::cout << "Refined estimated scale: " << s << std::endl;

    if (s < 0.0)
        return false;
    else
        return true;
}

void hwa_vis::vis_base::RefineGravity(hwa_vis::CamStateServer& camstates, Triple& g, Vector& x)
{
    Triple g0 = g.normalized() * 9.8;
    Triple lx, ly;
    //Vector x;
    int all_frame_count = camstates.size();
    int n_state = all_frame_count * 3 + 2 + 1;

    Matrix A{ n_state, n_state };
    A.setZero();
    Vector b{ n_state };
    b.setZero();

    hwa_vis::CamStateServer::iterator frame_i;
    hwa_vis::CamStateServer::iterator frame_j;
    for (int k = 0; k < 4; k++)
    {
        Matrix lxly(3, 2);
        lxly = TangentBasis(g0);
        int i = 0;
        for (frame_i = camstates.begin(); next(frame_i) != camstates.end(); frame_i++, i++)
        {
            frame_j = next(frame_i);

            Matrix tmp_A(6, 9);
            tmp_A.setZero();
            Vector tmp_b(6);
            tmp_b.setZero();

            // j -> i
            double dt = frame_i->second.pre_integration->sum_dt;

            tmp_A.block<3, 3>(0, 0) = -dt * SO3::Identity();
            tmp_A.block<3, 2>(0, 6) = frame_i->second.orientation_b.toRotationMatrix().transpose() * dt * dt / 2 * SO3::Identity() * lxly;
            tmp_A.block<3, 1>(0, 8) = frame_i->second.orientation_b.toRotationMatrix().transpose() * (frame_j->second.position - frame_i->second.position) / 100.0;
            tmp_b.block<3, 1>(0, 0) = frame_i->second.pre_integration->delta_p + frame_i->second.orientation_b.toRotationMatrix().transpose() * frame_j->second.orientation_b.toRotationMatrix() * frame_j->second.Tbc - frame_j->second.Tbc - frame_i->second.orientation_b.toRotationMatrix().transpose() * dt * dt / 2 * g0;

            tmp_A.block<3, 3>(3, 0) = -SO3::Identity();
            tmp_A.block<3, 3>(3, 3) = frame_i->second.orientation_b.toRotationMatrix().transpose() * frame_j->second.orientation_b.toRotationMatrix();
            tmp_A.block<3, 2>(3, 6) = frame_i->second.orientation_b.toRotationMatrix().transpose() * dt * SO3::Identity() * lxly;
            tmp_b.block<3, 1>(3, 0) = frame_i->second.pre_integration->delta_v - frame_i->second.orientation_b.toRotationMatrix().transpose() * dt * SO3::Identity() * g0;

            Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Zero();
            cov_inv.setIdentity();

            Matrix r_A = tmp_A.transpose() * cov_inv * tmp_A;
            Vector r_b = tmp_A.transpose() * cov_inv * tmp_b;

            A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
            b.segment<6>(i * 3) += r_b.head<6>();

            A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
            b.tail<3>() += r_b.tail<3>();

            A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
            A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
        }
        A = A * 1000.0;
        b = b * 1000.0;
        x = A.ldlt().solve(b);
        Vector dg = x.segment<2>(n_state - 3);
        g0 = (g0 + lxly * dg).normalized() * 9.8;
    }
    g = g0;
}

Matrix hwa_vis::vis_base::TangentBasis(Triple& g0)
{
    Triple b, c;
    Triple a = g0.normalized();
    Triple tmp(0, 0, 1);
    if (a == tmp)
        tmp << 1, 0, 0;
    b = (tmp - a * (a.transpose() * tmp)).normalized();
    c = a.cross(b);
    Matrix bc(3, 2);
    bc.block<3, 1>(0, 0) = b;
    bc.block<3, 1>(0, 1) = c;
    return bc;
}