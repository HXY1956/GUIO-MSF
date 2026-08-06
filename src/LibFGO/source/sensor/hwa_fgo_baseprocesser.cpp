#include "hwa_fgo_baseprocesser.h"
#include "hwa_base_filter.h"
#include "hwa_set_all.h"
using namespace hwa_fgo;
using namespace hwa_set;
using namespace hwa_base;

base_updater::base_updater(hwa_set::set_base* _gset, hwa_base::SENSOR_TYPE sensor) {
    switch (sensor) {
    case UWB:
        filter = str2updater(dynamic_cast<set_uwb*>(_gset)->filter());
        kappa_sig = dynamic_cast<set_uwb*>(_gset)->kappa_sig();
        alpha_sig = dynamic_cast<set_uwb*>(_gset)->alpha_sig();
        tau = dynamic_cast<set_uwb*>(_gset)->Tau();
        proc_noise = dynamic_cast<set_uwb*>(_gset)->proc_noise();
		proc_noise = proc_noise * proc_noise;
        g0 = dynamic_cast<set_uwb*>(_gset)->G0();
        e0 = dynamic_cast<set_uwb*>(_gset)->E0();
        max_iter = dynamic_cast<set_uwb*>(_gset)->max_iter();
        _num_particles = dynamic_cast<set_uwb*>(_gset)->num_particles();
        barrier = dynamic_cast<set_uwb*>(_gset)->barrior();
        dof1 = dynamic_cast<set_uwb*>(_gset)->dof1();
        dof2 = dynamic_cast<set_uwb*>(_gset)->dof2();
        max_res_norm = dynamic_cast<set_uwb*>(_gset)->max_res_norm();
        filter = str2updater(dynamic_cast<set_uwb*>(_gset)->filter());
        mode = str2proc_mode(dynamic_cast<set_uwb*>(_gset)->proc_mode());
        break;
    case GNSS:
        filter = str2updater(dynamic_cast<set_flt*>(_gset)->filter());
        kappa_sig = dynamic_cast<set_flt*>(_gset)->kappa_sig();
        alpha_sig = dynamic_cast<set_flt*>(_gset)->alpha_sig();
        tau = dynamic_cast<set_flt*>(_gset)->Tau();
        proc_noise = dynamic_cast<set_flt*>(_gset)->proc_noise();
        proc_noise = proc_noise * proc_noise;
        g0 = dynamic_cast<set_flt*>(_gset)->G0();
        e0 = dynamic_cast<set_flt*>(_gset)->E0();
        max_iter = dynamic_cast<set_flt*>(_gset)->max_iter();
        _num_particles = dynamic_cast<set_flt*>(_gset)->num_particles();
        barrier = dynamic_cast<set_flt*>(_gset)->barrior();
        dof1 = dynamic_cast<set_flt*>(_gset)->dof1();
        dof2 = dynamic_cast<set_flt*>(_gset)->dof2();
        max_res_norm = dynamic_cast<set_flt*>(_gset)->max_res_norm();
        filter = str2updater(dynamic_cast<set_flt*>(_gset)->filter());
        mode = str2proc_mode(dynamic_cast<set_flt*>(_gset)->proc_mode());
        break;
    case VISION:
        filter = str2updater(dynamic_cast<set_vis*>(_gset)->filter());
        kappa_sig = dynamic_cast<set_vis*>(_gset)->kappa_sig();
        alpha_sig = dynamic_cast<set_vis*>(_gset)->alpha_sig();
        tau = dynamic_cast<set_vis*>(_gset)->Tau();
        proc_noise = dynamic_cast<set_vis*>(_gset)->proc_noise();
        proc_noise = proc_noise * proc_noise;
        g0 = dynamic_cast<set_vis*>(_gset)->G0();
        e0 = dynamic_cast<set_vis*>(_gset)->E0();
        max_iter = dynamic_cast<set_vis*>(_gset)->max_iter();
        _num_particles = dynamic_cast<set_vis*>(_gset)->num_particles();
        barrier = dynamic_cast<set_vis*>(_gset)->barrior();
        dof1 = dynamic_cast<set_vis*>(_gset)->dof1();
        dof2 = dynamic_cast<set_vis*>(_gset)->dof2();
        max_res_norm = dynamic_cast<set_vis*>(_gset)->max_res_norm();
        filter = str2updater(dynamic_cast<set_vis*>(_gset)->filter());
        mode = str2proc_mode(dynamic_cast<set_vis*>(_gset)->proc_mode());
        break;
    case LIDAR:
        filter = str2updater(dynamic_cast<set_lidar*>(_gset)->filter());
        kappa_sig = dynamic_cast<set_lidar*>(_gset)->kappa_sig();
        alpha_sig = dynamic_cast<set_lidar*>(_gset)->alpha_sig();
        tau = dynamic_cast<set_lidar*>(_gset)->Tau();
        proc_noise = dynamic_cast<set_lidar*>(_gset)->proc_noise();
        proc_noise = proc_noise * proc_noise;
        g0 = dynamic_cast<set_lidar*>(_gset)->G0();
        e0 = dynamic_cast<set_lidar*>(_gset)->E0();
        max_iter = dynamic_cast<set_lidar*>(_gset)->max_iter();
        _num_particles = dynamic_cast<set_lidar*>(_gset)->num_particles();
        barrier = dynamic_cast<set_lidar*>(_gset)->barrior();
        dof1 = dynamic_cast<set_lidar*>(_gset)->dof1();
        dof2 = dynamic_cast<set_lidar*>(_gset)->dof2();
        max_res_norm = dynamic_cast<set_lidar*>(_gset)->max_res_norm();
        filter = str2updater(dynamic_cast<set_lidar*>(_gset)->filter());
        mode = str2proc_mode(dynamic_cast<set_lidar*>(_gset)->proc_mode());
        break;
    }
}