#include "hwa_msf_client.h"
#include "hwa_gnss_coder_bncobs.h"
#include "hwa_gnss_coder_bnccorr.h"
#include "hwa_gnss_coder_rinexo.h"
#include "hwa_gnss_coder_rinexn.h"
#include "hwa_gnss_coder_rinexc.h"
#include "hwa_gnss_coder_blq.h"
#include "hwa_gnss_coder_atx.h"
#include "hwa_gnss_coder_upd.h"
#include "hwa_gnss_coder_ifcb.h"
#include "hwa_gnss_coder_sp3.h"
#include "hwa_gnss_coder_biasinex.h"
#include "hwa_gnss_coder_biabernese.h"
#include "hwa_gnss_coder_poleut.h"
#include "hwa_gnss_coder_leapsecond.h"
#include "hwa_gnss_coder_aug.h"
#include "hwa_gnss_coder_dvpteph405.h"
#include "hwa_base_io_serial.h"

using namespace std;
using namespace std::chrono;
using namespace hwa_set;
using namespace hwa_base;
using namespace hwa_msf;

void catch_signal(int) { std::cout << "Program interrupted by Ctrl-C [SIGINT,2]\n"; }

int main(int argc, char** argv)
{
    signal(SIGINT, catch_signal);

    auto gset = std::make_shared<set_msf>();
    gset->app("hwa_msf", "1.0.0", "$Rev: 2448 $", "(@whu,edu,cn)", __DATE__, __TIME__);
    gset->arg(argc, argv, true, false);

    auto log_type = gset->log_type();
    auto log_level = gset->log_level();
    auto log_name = gset->log_name();
    auto log_pattern = gset->log_pattern();
    spdlog::set_level(log_level);
    spdlog::set_pattern(log_pattern);
    spdlog::flush_on(spdlog::level::err);
    base_rtlog great_log = base_rtlog(log_type, log_level, log_name);
    auto my_logger = great_log.spdlog();

    bool isBase = false;
    if (gset->list_base().size()) isBase = true;
    bool isClient = dynamic_cast<set_npp*>(gset.get())->isClient();
    // Prepare site list from gset
    string site_rover;
    if (isClient)  site_rover = dynamic_cast<set_gen*>(gset.get())->list_rover()[0];
    set<string> sites = dynamic_cast<set_gen*>(gset.get())->recs();
    // Prepare input files list form gset
    multimap<IFMT, string> inp = gset->inputs_all();
    // Get sample intval from gset-> if not, init with the default value
    int sample = int(dynamic_cast<set_gen*>(gset.get())->sampling());
    if (!sample) sample = int(dynamic_cast<set_gen*>(gset.get())->sampling_default());

    base_time beg = dynamic_cast<set_gen*>(gset.get())->beg();
    base_time end = dynamic_cast<set_gen*>(gset.get())->end();

    // DECLARATIONS/INITIALIZATIONS
    // gobs for the obs data
    gnss_all_obs* gobs = new gnss_all_obs();  gobs->spdlog(my_logger); gobs->gset(gset.get());
    // gallnav for all the navigation data, gorb = gorbit data
    gnss_all_prec* gorb = new gnss_all_prec(); gorb->spdlog(my_logger);
    // gpcv for the atx data read from the atx file
    gnss_all_pcv* gpcv = 0; if (gset->input_size("atx") > 0) { gpcv = new gnss_all_pcv;  gpcv->spdlog(my_logger); }
    // gotl for the blq data read from the blq file, which will be used for Ocean tidal corrections
    gnss_all_otl* gotl = 0; if (gset->input_size("blq") > 0) { gotl = new gnss_all_otl;  gotl->spdlog(my_logger); }
    // gbia for the DCB data read from the biasinex and bianern files
    gnss_all_bias* gbia = 0;
    if (gset->input_size("biasinex") > 0 || gset->input_size("bias") > 0) {
        gbia = new gnss_all_bias; gbia->spdlog(my_logger);
    }
    gnss_all_obj* base_obj = new gnss_all_obj(my_logger, gpcv, gotl); base_obj->spdlog(my_logger);

    base_data* base_data = 0;
    gnss_data_upd* gupd = nullptr; if (gset->input_size("upd") > 0) { gupd = new gnss_data_upd;  gupd->spdlog(my_logger); }
    gnss_data_aug* gaug = new gnss_data_aug;
    gnss_data_navde* gde = new gnss_data_navde;
    gnss_data_poleut* gerp = new gnss_data_poleut;
    gnss_data_leapsecond* gleap = new gnss_data_leapsecond;
    gnss_data_ifcb* gifcb = nullptr; if (gset->input_size("ifcb") > 0) { gifcb = new gnss_data_ifcb;  gifcb->spdlog(my_logger); }
    ins_data* gimu = new ins_data(); gimu->spdlog(my_logger);
    uwb_data* guwb = new uwb_data(); guwb->spdlog(my_logger);
    vis_data* gimg = new vis_data(gset.get()); gimg->spdlog(my_logger);
    // lidar_data* glidar = new lidar_data(); glidar->spdlog(my_logger);

    base_time runepoch(base_time::GPS);
    base_time lstepoch(base_time::GPS);
    if (gset->realtime() == true) {
        gorb->use_clknav(true);
        gorb->use_clkrnx(false);
        gorb->use_posnav(true);
        gorb->use_clksp3(false);
    }
    else {
        if (gset->input_size("sp3") == 0 && gset->input_size("rinexc") == 0)
        {
            gorb->use_clknav(true);
            gorb->use_posnav(true);
        }
        else if (gset->input_size("sp3") > 0 && gset->input_size("rinexc") == 0)
        {
            gorb->use_clksp3(true);
        }
    }

    if (gset->realtime() == false && !isBase)
    {
        if (gset->input_size("sp3") == 0 &&
            gset->input_size("rinexc") == 0 &&
            gset->input_size("rinexo") == 0
            ) {
            SPDLOG_LOGGER_INFO(my_logger, "main", "Error: incomplete input: rinexo + rinexc + sp3");
            gset->usage();
        }
    }

    set<string>::const_iterator itOBJ;
    set<string> obj = dynamic_cast<set_rec*>(gset.get())->objects();
    for (itOBJ = obj.begin(); itOBJ != obj.end(); ++itOBJ) {
        string name = *itOBJ;
        shared_ptr<gnss_data_rec> rec = dynamic_cast<set_rec*>(gset.get())->grec(name, my_logger);
        base_obj->add(rec);
    }
    vector<base_coder*> base_coder_obj;
    vector<base_io*> base_io_obj;
    vector<thread> base_thread;
    base_io* unique_io = 0;
    base_coder* unique_coder = 0;

    // DATA READING
    multimap<IFMT, string>::const_iterator itINP = inp.begin();
    for (size_t i = 0; i < inp.size() && itINP != inp.end(); ++i, ++itINP)
    {
        // Get the file format/path, which will be used in decoder
        IFMT   ifmt(itINP->first);
        string path(itINP->second);
        string id("ID" + base_type_conv::int2str(i));

        // For different file format, we prepare different data container and decoder for them.
        // Decode bnc obs data
        if (ifmt == IFMT::BNCOBS_INP) { base_data = gobs; unique_coder = new gnss_coder_bncobs(gset.get(), "", 40960); }
        else if (ifmt == IFMT::BNCBRDC_INP) { base_data = gorb;  unique_coder = new gnss_coder_rinexn(gset.get(), "", 4096); }
        else if (ifmt == IFMT::BNCCORR_INP) { base_data = gorb;  unique_coder = new gnss_coder_bnccorr(gset.get(), "", 4096); }
        else if (ifmt == IFMT::SP3_INP) { base_data = gorb; unique_coder = new gnss_coder_sp3(gset.get(), "", 8172); }
        else if (ifmt == IFMT::RINEXO_INP) { base_data = gobs; unique_coder = new gnss_coder_rinexo(gset.get(), "", 4096); }
        else if (ifmt == IFMT::RINEXC_INP) { base_data = gorb; unique_coder = new gnss_coder_rinexc(gset.get(), "", 4096); }
        else if (ifmt == IFMT::RINEXN_INP) { base_data = gorb; unique_coder = new gnss_coder_rinexn(gset.get(), "", 4096); }
        else if (ifmt == IFMT::ATX_INP) { base_data = gpcv; unique_coder = new gnss_coder_atx(gset.get(), "", 4096); }
        else if (ifmt == IFMT::BLQ_INP) { base_data = gotl; unique_coder = new gnss_coder_blq(gset.get(), "", 4096); }
        else if (ifmt == IFMT::UPD_INP) {
            base_data = gupd;
            if (gset->realtime() == true) unique_coder = new gnss_coder_bnccorr(gset.get(), "", 4096);
            else                        unique_coder = new gnss_coder_upd(gset.get(), "", 4096);
        }
        else if (ifmt == IFMT::BIASINEX_INP) { base_data = gbia; unique_coder = new gnss_coder_biasinex(gset.get(), "", 20480); }
        else if (ifmt == IFMT::BIAS_INP) { base_data = gbia; unique_coder = new gnss_coder_biabernese(gset.get(), "", 20480); }
        else if (ifmt == IFMT::SINEX_INP) { unique_coder = new gnss_coder_sinex(gset.get(), "", 20480); }
        else if (ifmt == IFMT::DE_INP) { base_data = gde; unique_coder = new gnss_coder_dvpteph405(gset.get(), "", 4096); }
        else if (ifmt == IFMT::EOP_INP) { base_data = gerp; unique_coder = new gnss_coder_poleut(gset.get(), "", 4096); }
        else if (ifmt == IFMT::LEAPSECOND_INP) { base_data = gleap; unique_coder = new gnss_coder_leapsecond(gset.get(), "", 4096); }
        else if (ifmt == IFMT::IFCB_INP) { base_data = gifcb; unique_coder = new gnss_coder_ifcb(gset.get(), "", 4096); }
        else if (ifmt == IFMT::AUG_INP) {
            base_data = gaug;
            if (path.substr(0, 6) == "tcp://") unique_coder = new gnss_coder_bnccorr(gset.get(), "", 4096);
            else
                unique_coder = new gnss_coder_aug(gset.get(), "", 4096);
        }
        // else if (ifmt == IFMT::LIDAR_INP) { base_data = glidar; unique_coder = new lidar_coder(gset.get(), "", 4096); }
        else if (ifmt == IFMT::IMU_INP) { base_data = gimu; unique_coder = new ins_coder(gset.get(), "", 40960); }
        else if (ifmt == IFMT::UWBMSG_INP) { base_data = guwb; unique_coder = new uwb_coder(gset.get(), 4096); guwb->add_nodes(dynamic_cast<set_uwb*>(gset.get())->anchor_list());}
        else if (ifmt == IFMT::IMAGE_INP) { base_data = gimg; unique_coder = new vis_coder(gset.get(), "", 40960); }
        //else if (ifmt == TAG_INP) { base_data = gtag; unique_coder = new t_tagfile(gset.get(), "", 4096); }
        else {
            SPDLOG_LOGGER_INFO(my_logger, "main", "Error: unrecognized format " + base_type_conv::int2str(int(ifmt)));
            base_data = 0;
        }
        // Check the file path
        if (path.substr(0, 7) == "file://") {
            SPDLOG_LOGGER_INFO(my_logger, "main", "path is file!");
            unique_io = new base_file(my_logger);
            unique_io->spdlog(my_logger);
            unique_io->path(path);
        }
        if (path.substr(0, 6) == "tcp://") {
            SPDLOG_LOGGER_INFO(my_logger, "main", "path is tcp host!");
            unique_io = new base_io_tcp(path);
            unique_io->spdlog(my_logger);
        }
        if (path.substr(0, 9) == "serial://") {
            SPDLOG_LOGGER_INFO(my_logger, "main", "path is serial!");
            unique_io = new base_io_serial(path);
            unique_io->spdlog(my_logger);
        }
        // READ DATA FROM FILE
        if (unique_coder) {
            // Put the file into base_coder_obj
            unique_coder->clear();
            unique_coder->path(path);
            unique_coder->spdlog(my_logger);
            // Put the data container into base_coder_obj
            unique_coder->add_data(id, base_data);
            if (ifmt != IFMT::IMU_INP && ifmt != IFMT::ODO_INP && ifmt != IFMT::UWBMSG_INP && ifmt != IFMT::LIDAR_INP)
            {
                unique_coder->add_data("OBJ", base_obj);
            }

            if (ifmt == IFMT::BNCCORR_INP)
            {
                unique_coder->add_data("BIAS", gbia);
            }
            // Put the base_coder_obj into the base_io
            // Note, base_coder_obj contain the base_data and base_io contain the base_coder_obj
            unique_io->coder(unique_coder);

            if (gset->realtime() == true &&
                (ifmt >= IFMT::BNCOBS_INP && ifmt <= IFMT::BNCBRDC_INP ||
                    (ifmt == IFMT::UPD_INP || ifmt == IFMT::AUG_INP) && path.substr(0, 6) == "tcp://") ||
                (ifmt == IFMT::AUG_INP && path.substr(0, 9) == "caster://"))
            {
                base_io_obj.push_back(unique_io);
                base_coder_obj.push_back(unique_coder);
            }
            else {
                runepoch = base_time::current_time(base_time::GPS);
                // Read the data from file here
                auto start = std::chrono::high_resolution_clock::now();
                unique_io->run_read();
                auto end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> duration = end - start;
                std::cout << hwa_set::set_inp::ifmt2str(ifmt) <<"Time used: " << duration.count() << " seconds" << std::endl;
                lstepoch = base_time::current_time(base_time::GPS);
                // Write the information of reading process to log file
                SPDLOG_LOGGER_INFO(my_logger, "main", "READ: " + path + " time: "
                    + base_type_conv::dbl2str(lstepoch.diff(runepoch)) + " sec");
                // Delete 
                delete unique_io;
                delete unique_coder;
            }
        }
    }

    gimu->sort_IMU();
    base_obj->read_satinfo(beg);
    base_obj->sync_pcvs();
    // add all data
    base_all_proc* data = new base_all_proc();
    if (gobs)
        data->Add_Data(base_data::type2str(gobs->id_type()), gobs);
    if (gorb)
        data->Add_Data(base_data::type2str(gorb->id_type()), gorb);
    if (base_obj)
        data->Add_Data(base_data::type2str(base_obj->id_type()), base_obj);
    if (gbia)
        data->Add_Data(base_data::type2str(gbia->id_type()), gbia);
    if (gotl)
        data->Add_Data(base_data::type2str(gotl->id_type()), gotl);
    if (gde)
        data->Add_Data(base_data::type2str(gde->id_type()), gde);
    if (gerp)
        data->Add_Data(base_data::type2str(gerp->id_type()), gerp);
    if (gleap)
        data->Add_Data(base_data::type2str(gleap->id_type()), gleap);
    if (gimu)
        data->Add_Data(base_data::type2str(gimu->id_type()), gimu);
    if (guwb)
        data->Add_Data(base_data::type2str(guwb->id_type()), guwb);
    if (gimg)
        data->Add_Data(base_data::type2str(gimg->id_type()), gimg);
    if (gupd && dynamic_cast<set_amb*>(gset.get())->fix_mode() != FIX_MODE::NO && !isBase)
    {
        if (!isClient) { data->Add_Data(base_data::type2str(gupd->id_type()), gupd); };
    }
    int frequency = dynamic_cast<set_gproc*>(gset.get())->frequency();
    set<string> system = dynamic_cast<set_gen*>(gset.get())->sys();
    if (frequency == 3 && system.find("GPS") != system.end() && !isBase)
    {
        data->Add_Data(base_data::type2str(gifcb->id_type()), gifcb);
    }
    if (isClient) { data->Add_Data(base_data::type2str(gaug->id_type()), gaug); };
    size_t read_num = 0;
    if (gset->realtime() == true) {
        for (size_t i = 0; i < base_io_obj.size(); i++) {
            SPDLOG_LOGGER_INFO(my_logger, "main", "Multi-thread receiving and reading started");
            base_thread.push_back(thread(&base_io::run_read, base_io_obj[i]));
        }
        read_num = base_thread.size();
        for (size_t i = 0; i < read_num; ++i) { base_thread[i].detach(); }
        if (!isBase) { while (!gorb->corr_avali())base_time::gmsleep(1000); }
        base_time::gmsleep(sample * 2000);
    }

    auto tic_start = system_clock::now();
    int i = 0, nsite = sites.size();
    std::map<int, std::unique_ptr<msf_client>> vgipn;
    if (isBase || isClient) nsite = gset->list_rover().size();
    set<string>::iterator it = sites.begin();
    while (i < nsite) {
        string site_base = "";
        string site = *it;
        if (isBase) {
            site_base = (gset->list_base())[i];
            site = (gset->list_rover())[i];
            if (!isClient && (gobs->beg_obs(site_base) == LAST_TIME || gobs->end_obs(site_base) == FIRST_TIME ||
                site_base.empty() || gobs->isSite(site_base) == false)) {
                SPDLOG_LOGGER_INFO(my_logger, "main", "No two site/data for processing!");
                i++;
                continue;
            }
        }

        if (gset->realtime() == true) beg = gobs->beg_obs(site);
        runepoch = base_time::current_time(base_time::GPS);

        vgipn[i] = std::make_unique<msf_client>(site, site_base, beg, end, gset, my_logger, data);
        vgipn[i]->ProcessBatchFB();

        SPDLOG_LOGGER_INFO(my_logger, "main", site_base + site + "PVT/INS processing finished : duration  "
            + base_type_conv::dbl2str(lstepoch.diff(runepoch)) + " sec");

        if (!isBase) it++;
        i++;
    }

    for (size_t i = 0; i < base_io_obj.size(); ++i) { delete base_io_obj[i]; }; base_io_obj.clear();
    for (size_t i = 0; i < base_coder_obj.size(); ++i) { delete base_coder_obj[i]; }; base_coder_obj.clear();

    if (gobs) delete gobs;
    if (gpcv) delete gpcv;
    if (gotl) delete gotl;
    if (base_obj) delete base_obj;
    if (gorb) delete gorb;
    if (gbia) delete gbia;
    if (gde)  delete gde;
    if (gerp)  delete gerp;
    if (gupd)  delete gupd;
    if (gifcb)  delete gifcb;
    if (gleap)  delete gleap;
    if (gaug) delete gaug;
    if (data)  delete data;
    if (guwb) delete guwb;
    if (gimu) delete gimu;
    // if (glidar) delete glidar;
    if (gimg) delete gimg;

    auto tic_end = system_clock::now();
    auto duration = duration_cast<microseconds>(tic_end - tic_start);
    cout << "Spent" << double(duration.count()) * microseconds::period::num / microseconds::period::den << " seconds." << endl;

}

