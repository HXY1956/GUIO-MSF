#include "hwa_pg_database.h"

#include "hwa_pg_internal.h"
#include <chrono>
#include <iostream>

namespace hwa_pg {

struct PGDatabase::PGDatabaseImpl {
    std::shared_ptr<BriefVocabulary> voc;
    BriefDatabase db;
    std::vector<int> entry_to_kfid;                       // DBoW entry id -> keyframe id
    std::vector<std::vector<DBoW2::FBrief::TDescriptor>> pending;  // pre-vocab buffer
    std::vector<int> pending_kfid;
    int alive = 0;                                        // live (non-pruned) entries
    bool ready = false;
};

PGDatabase::PGDatabase() = default;
PGDatabase::~PGDatabase() = default;

bool PGDatabase::loadVocabulary(const std::string& path)
{
    if (path.empty()) return false;
    if (!impl_) impl_ = std::make_unique<PGDatabaseImpl>();
    try {
        bool is_bin = path.size() >= 4 &&
            path.compare(path.size() - 4, 4, ".bin") == 0;
        if (is_bin) {
            impl_->voc = std::make_shared<BriefVocabulary>(path);   // VINS loadBin
        } else {
            impl_->voc = std::make_shared<BriefVocabulary>();
            impl_->voc->load(path);                                 // text / YAML
        }
        if (!impl_->voc->empty()) {
            impl_->db.setVocabulary(*impl_->voc, false, 0);
            impl_->ready = true;
            std::cout << "[PG] loaded DBoW2 vocabulary: " << path << std::endl;
            return true;
        }
    } catch (const std::exception& e) {
        std::cerr << "[PG] failed to load vocabulary " << path << ": " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "[PG] failed to load vocabulary " << path << std::endl;
    }
    impl_->voc.reset();
    impl_->ready = false;
    return false;
}

bool PGDatabase::trainVocabulary(const std::vector<std::shared_ptr<PGKeyFrame>>& kfs,
                                 int k, int L)
{
    if (!impl_) impl_ = std::make_unique<PGDatabaseImpl>();

    std::vector<std::vector<DBoW2::FBrief::TDescriptor>> training;
    for (const auto& kf : kfs) {
        if (!kf || !kf->dbow || kf->dbow->brief_descriptors.empty()) continue;
        training.push_back(kf->dbow->brief_descriptors);
    }
    if (training.size() < 10) return false;

    impl_->voc = std::make_shared<BriefVocabulary>(k, L,
            DBoW2::TF_IDF, DBoW2::L1_NORM);
    impl_->voc->create(training);
    impl_->db.setVocabulary(*impl_->voc, false, 0);
    impl_->ready = true;
    // flush keyframes buffered before the vocabulary was ready
    for (size_t i = 0; i < impl_->pending.size(); ++i) {
        DBoW2::EntryId entry = impl_->db.add(impl_->pending[i]);
        impl_->alive++;
        if ((int)impl_->entry_to_kfid.size() <= entry)
            impl_->entry_to_kfid.resize(entry + 1, -1);
        impl_->entry_to_kfid[entry] = impl_->pending_kfid[i];
    }
    impl_->pending.clear();
    impl_->pending_kfid.clear();
    std::cout << "[PG] trained runtime DBoW2 vocabulary on "
              << training.size() << " keyframes (k=" << k << ", L=" << L << ")" << std::endl;
    return true;
}

bool PGDatabase::hasVocabulary() const
{
    return impl_ && impl_->voc && !impl_->voc->empty();
}

void PGDatabase::addKeyFrame(const std::shared_ptr<PGKeyFrame>& kf)
{
    if (!kf) return;
    if (!impl_) impl_ = std::make_unique<PGDatabaseImpl>();
    if (!kf->dbow) kf->dbow = std::make_shared<PGKeyFrameDBoW>();
    if (kf->dbow->brief_descriptors.empty()) return;

    if (!impl_->ready) {
        impl_->pending.push_back(kf->dbow->brief_descriptors);
        impl_->pending_kfid.push_back(kf->id);
        return;
    }

    impl_->voc->transform(kf->dbow->brief_descriptors, kf->dbow->bow, kf->dbow->feat, 4);
    DBoW2::EntryId entry = impl_->db.add(kf->dbow->brief_descriptors);
    impl_->alive++;
    kf->dbow->entry = (int)entry;
    if ((int)impl_->entry_to_kfid.size() <= entry)
        impl_->entry_to_kfid.resize(entry + 1, -1);
    impl_->entry_to_kfid[entry] = kf->id;
}

void PGDatabase::deleteEntry(int entry_id)
{
    if (!impl_ || !impl_->ready || entry_id < 0) return;
    impl_->db.delete_entry((DBoW2::EntryId)entry_id);
    if (entry_id < (int)impl_->entry_to_kfid.size())
        impl_->entry_to_kfid[entry_id] = -1;
    if (impl_->alive > 0)
        impl_->alive--;
}

void PGDatabase::rebaseKfIds(int offset)
{
    if (!impl_ || offset <= 0) return;
    for (auto& v : impl_->entry_to_kfid)
        if (v >= 0)
            v -= offset;
}

size_t PGDatabase::size() const
{
    return impl_ ? (size_t)impl_->alive : 0;
}

size_t PGDatabase::nextEntryId() const
{
    // DBoW entry ids are never reused: delete_entry() does not decrement
    // m_nentries, so the next add() will allocate exactly db.size().
    return impl_ ? (size_t)impl_->db.size() : 0;
}

std::vector<std::pair<int, double>> PGDatabase::queryCandidates(
    const std::shared_ptr<PGKeyFrame>& kf, int topN, int maxId) const
{
    std::vector<std::pair<int, double>> res;
    if (!impl_ || !impl_->ready || !kf || !kf->dbow || kf->dbow->brief_descriptors.empty())
        return res;

    DBoW2::QueryResults ret;
    try {
        const auto t0 = std::chrono::steady_clock::now();
        impl_->db.query(kf->dbow->brief_descriptors, ret, topN, maxId);
        const double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        // Diagnostic: DBoW2 queryL1 scans the whole inverted file (only the
        // most recent maxId entries are skipped), so cost grows linearly with
        // the number of live database entries. If this log shows growing times
        // and entry counts, raise max_pg_keyframes or tighten thinning.
        if (ms > 30.0)
            std::cout << "[PG] slow query " << ms << " ms, entries="
                      << size() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[PG] DBoW query failed: " << e.what() << std::endl;
        return res;
    }
    for (const auto& r : ret) {
        int kfid = (r.Id >= 0 && r.Id < (int)impl_->entry_to_kfid.size())
                       ? impl_->entry_to_kfid[r.Id] : -1;
        if (kfid >= 0)
            res.emplace_back(kfid, r.Score);
    }
    return res;
}

} // namespace hwa_pg
