#include <iostream>
#include <stdlib.h>
#include "hwa_gnss_data_monit.h"

namespace hwa_gnss
{
    /* ----------
     * constructor
     */
    gnss_data_monit::gnss_data_monit(const std::string &id)
        : _moni_id(id)
    {
    }

    /* ----------
     * destructor
     */
    gnss_data_monit::~gnss_data_monit()
    {
    }

    /* ----------
     * basic class for monitoring purpose
     */
    void gnss_data_monit::show(std::ostringstream &os, int verb)
    {
        os << _moni_id << " - method not implemented\n";
        return;
    }

} // namespace
