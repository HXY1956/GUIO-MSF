#ifndef hwa_gnss_data_trn_H
#define hwa_gnss_data_trn_H

#include <stdio.h>
#include <string>
#include "hwa_gnss_data_Obj.h"
#include "hwa_base_data.h"
#include "hwa_gnss_data_rxnhdr.h"

namespace hwa_gnss
{

    /** @brief class for gtrn. */
    class gnss_data_trn : public gnss_data_obj
    {

    public:
        /** @brief default constructor. */
        gnss_data_trn();

        /**
         * @brief Construct a new t gtrn object
         * 
         * @param spdlog 
         */
        gnss_data_trn(base_log spdlog);
        /** @brief default destructor. */
        virtual ~gnss_data_trn();

        /** @brief header pair. */
        typedef std::pair<std::string, gnss_data_rxnhdr> t_header_pair;

        /** @brief header. */
        typedef std::vector<t_header_pair> t_header;

        /** @brief header. */
        virtual void header(const gnss_data_rxnhdr &hdr, std::string path);

        /** @brief overried header. */
        virtual gnss_data_rxnhdr header(std::string path) const;

        /** @brief headers. */
        virtual t_header headers() const;

        /** @brief get the value of isrec. */
        virtual bool isrec() override { return true; }

        /** @brief get the value of istrn. */
        virtual bool istrn() override { return false; }

        /** @brief get the value of channel. */
        virtual void channel(int chk) override;

        /** @brief channel override. */
        virtual int channel() const;

    protected:
        gnss_data_rxnhdr _header(const base_time &epo); ///< a rinex header

        t_header _headers; ///< map of rinex header information
        int _channel;      ///< temporarily only one value. Must be enhance via _mapchk
    private:
    };

} // namespace
#endif
