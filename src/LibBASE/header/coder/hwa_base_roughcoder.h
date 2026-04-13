#ifndef hwa_base_roughcoder_h
#define hwa_base_roughcoder_h
#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>

namespace hwa_base {
	class base_coder_rough {
	public:
		base_coder_rough(std::string _file_path) : file_path(_file_path),
		bufferleft(""),
		chunksize(102400){};
		virtual ~base_coder_rough() = default;
		virtual bool decode_data() = 0;
		virtual bool decode_head() = 0;
		virtual bool read() = 0;

	protected:
		std::fstream file;
		std::string file_path;
		int chunksize;
		std::string bufferleft;
	};
}

#endif