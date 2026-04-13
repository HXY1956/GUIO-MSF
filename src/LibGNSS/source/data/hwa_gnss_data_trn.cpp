#include "hwa_gnss_data_trn.h"

using namespace std;

namespace hwa_gnss
{
    // constructor
    // ----------
    gnss_data_trn::gnss_data_trn()
        : gnss_data_obj(),
          _channel(255)
    // base_data::id_type(TRN) // NEFUNGUJE?
    {
        //  cout << "CONSTRUCTOR gnss_data_trn \n"; cout.flush();
        id_type(TRN);
    }

    gnss_data_trn::gnss_data_trn(base_log spdlog)
        : gnss_data_obj(spdlog),
          _channel(255)
    {
        id_type(TRN);
    }
    // destructor
    // ----------
    gnss_data_trn::~gnss_data_trn()
    {
        //  boost::mutex::scoped_lock lock(_mutex);

        //  _mapchk.clear();
        //  cout << "DESTRUCTOR gnss_data_trn \n"; cout.flush();
    }

    // add rinex header
    // ----------------
    void gnss_data_trn::header(const gnss_data_rxnhdr &hdr, string path)
    {
        t_header_pair pair = make_pair(path, hdr);
        _headers.push_back(pair);
    }

    // get all rinex header
    // -------------------
    gnss_data_trn::t_header gnss_data_trn::headers() const
    {
        return _headers;
    }

    // get one rinex header
    // -------------------
    gnss_data_rxnhdr gnss_data_trn::header(string path) const
    {
        gnss_data_rxnhdr rxnhdr;
        for (auto it = _headers.begin(); it != _headers.end(); ++it)
        {
            if (it->first.compare(path) == 0)
            {
                return it->second;
            }
        }

        return rxnhdr;
    }

    // set channel
    // ----------
    void gnss_data_trn::channel(int chk)
    {
        _channel = chk;
        return;
    }

    // get channel
    // ----------
    int gnss_data_trn::channel() const
    {
        int tmp = _channel;
        return tmp;
    }

} // namespace
