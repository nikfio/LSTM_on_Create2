#include <cassert>
#include <string>
#include <vector>
#include <iostream>
#include <cstdio>

#include "leveldb/db.h"
#include "lmdb.h"

using std::string;
using std::cout;
using std::vector;
using std::endl;

int main(int argc, char **argv) {

cout << "Reading db util called for type " << argv[2] << endl;



	if( strcmp(argv[2], "leveldb") == 0 ) {

		leveldb::DB* database;
		leveldb::Options options;
		options.create_if_missing = true;
		leveldb::Status status = leveldb::DB::Open(options, argv[1], &database);
		assert(status.ok());

		leveldb::Iterator* it = database->NewIterator(leveldb::ReadOptions());
		for (it->SeekToFirst(); it->Valid(); it->Next()) {
		  cout << it->key().ToString() << ": "  << it->value().ToString() << endl;
		}
		assert(it->status().ok());  // Check for any errors found during the scan
		delete it;
		delete database;

	}
	else if ( !strcmp(argv[2], "lmdb") ) {
		
		int rc;
		MDB_env *env;
		MDB_dbi dbi;
		MDB_val key, data;
		MDB_txn *txn;
		MDB_cursor *cursor;
		char sval[32];

		rc = mdb_env_create(&env);
		rc = mdb_env_open(env, "./testdb", 0, 0664);
		rc = mdb_txn_begin(env, NULL, 0, &txn);
		rc = mdb_open(txn, NULL, 0, &dbi);

		key.mv_size = sizeof(int);
		key.mv_data = sval;
		data.mv_size = sizeof(sval);
		data.mv_data = sval;

		sprintf(sval, "%03x %d foo bar", 32, 3141592);
		rc = mdb_put(txn, dbi, &key, &data, 0);
		rc = mdb_txn_commit(txn);
		if (rc) {
			fprintf(stderr, "mdb_txn_commit: (%d) %s\n", rc, mdb_strerror(rc));
			goto leave;
		}
		rc = mdb_txn_begin(env, NULL, MDB_RDONLY, &txn);
		rc = mdb_cursor_open(txn, dbi, &cursor);
		while ((rc = mdb_cursor_get(cursor, &key, &data, MDB_NEXT)) == 0) {
			printf("key: %p %.*s, data: %p %.*s\n",
				key.mv_data,  (int) key.mv_size,  (char *) key.mv_data,
				data.mv_data, (int) data.mv_size, (char *) data.mv_data);
		}
		mdb_cursor_close(cursor);
		mdb_txn_abort(txn);
		leave:
		mdb_close(env, dbi);
		mdb_env_close(env);

	}


}
