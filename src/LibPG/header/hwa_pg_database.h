#ifndef HWA_PG_DATABASE_H
#define HWA_PG_DATABASE_H

#include "hwa_pg_types.h"
#include <memory>
#include <string>
#include <vector>

namespace hwa_pg {

// Loop-closure retrieval database built on DBoW2 (BRIEF vocabulary).
// Mirrors VINS-Mono pose_graph database usage:
//   - each keyframe is converted into a bag-of-words vector,
//   - query is performed BEFORE the current frame is added,
//   - candidate keyframes are verified geometrically by the caller.
class PGDatabase {
public:
    PGDatabase();
    ~PGDatabase();
    PGDatabase(const PGDatabase&) = delete;
    PGDatabase& operator=(const PGDatabase&) = delete;

    // Load a pre-trained vocabulary: text (ORBvoc.txt) or binary (.bin).
    bool loadVocabulary(const std::string& path);

    // Train a vocabulary at runtime from the first keyframes (fallback).
    bool trainVocabulary(const std::vector<std::shared_ptr<PGKeyFrame>>& kfs,
                         int k = 10, int L = 6);

    bool hasVocabulary() const;

    // Add a keyframe (BRIEF descriptors -> BoW -> database).
    void addKeyFrame(const std::shared_ptr<PGKeyFrame>& kf);

    // Remove a previously added entry (keeps the database bounded). Entries are
    // deleted from the DBoW inverted index; their ids are never reused.
    void deleteEntry(int entry_id);

    // After the pose graph drops its oldest keyframes, keyframe ids are rebased
    // (id -= offset). Keeps the entry_id -> keyframe_id mapping consistent.
    void rebaseKfIds(int offset);

    // Number of live (non-deleted) entries.
    size_t size() const;

    // Id of the next entry that addKeyFrame() will allocate (== historical
    // number of entries ever added). Used to compute DBoW max_id filters that
    // survive pruning, because entry ids are never reused.
    size_t nextEntryId() const;

    // Query candidates (keyframe_id, score), ignoring entries with id > maxId.
    std::vector<std::pair<int, double>> queryCandidates(
        const std::shared_ptr<PGKeyFrame>& kf, int topN = 4, int maxId = 0) const;

private:
    struct PGDatabaseImpl;
    std::unique_ptr<PGDatabaseImpl> impl_;
};

} // namespace hwa_pg

#endif
