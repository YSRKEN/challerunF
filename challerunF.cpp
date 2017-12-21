// 条件分岐を減らし、より素直なコードに書き直した

//#pragma execution_character_set("utf-8")

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using std::cout;
using std::endl;
using std::ostream;
using std::string;
using std::vector;

class StopWatch {
	std::chrono::system_clock::time_point start_time;
	std::chrono::system_clock::time_point end_time;
public:
	// ストップウォッチを開始
	void Start() noexcept{ start_time = std::chrono::system_clock::now(); }
	// ストップウォッチを停止
	void Stop() noexcept{ end_time = std::chrono::system_clock::now(); }
	// 経過時間を返す
	long long ElapsedNanoseconds() const noexcept {
		return std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
	}
	long long ElapsedMicroseconds() const noexcept {
		return std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
	}
	long long ElapsedMilliseconds() const noexcept {
		return std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
	}
};

// ソフトウェアの動作設定
class Setting {
	// 問題のファイル名
	string file_name_;
	// スタート地点
	int start_position_ = -1;
	// ゴール地点
	int goal_position_ = -1;
	// 問題を解くモードか？(trueならソルバーモード、falseなら分割モード)
	bool solver_flg_ = true;
	// ソルバーモードの際のスレッド数、分割モードの際の分割数
	unsigned int split_count_ = 1;
public:
	// コンストラクタ
	Setting(int argc, char* argv[]) {
		// 引数の数がおかしい場合は例外を投げる
		if (argc < 4)
			throw "引数の数が少なすぎます。";
		// 問題のファイル名を読み取る
		file_name_ = argv[1];
		// スタート地点を読み取る
		start_position_ = std::stoi(argv[2]);
		// ゴール地点を読み取る
		goal_position_ = std::stoi(argv[3]);
		// オプション部分を読み取る
		if (argc < 5)
			return;
		{
			int option = std::stoi(argv[4]);
			if (option != 0) {
				// ソルバーモード
				solver_flg_ = true;
				split_count_ = std::abs(option);
				if (split_count_ < 1)
					split_count_ = 1;
			}
			else {
				// 分割モード
				solver_flg_ = false;
				if (argc >= 6) {
					split_count_ = std::abs(std::stoi(argv[5]));
					if (split_count_ <= 1)
						split_count_ = 2;
				}
			}
		}
	}
	// getter
	string file_name() const noexcept { return file_name_; }
	int start_position() const noexcept { return start_position_; }
	int goal_position() const noexcept { return goal_position_; }
	bool solver_flg() const noexcept { return solver_flg_; }
	unsigned int split_count() const noexcept { return split_count_; }
	// 出力用
	friend ostream& operator << (ostream& os, const Setting& setting) noexcept{
		os << "【設定】" << endl;
		os << "・ファイル名：" << setting.file_name_ << endl;
		os << "・スタート地点：" << setting.start_position_ << endl;
		os << "・ゴール地点：" << setting.goal_position_ << endl;
		if(setting.solver_flg_){
			os << "・動作モード：ソルバーモード" << endl;
			os << "・動作スレッド数：" << setting.split_count_ << endl;
		}
		else {
			os << "・動作モード：分割モード" << endl;
			os << "・ファイル分割数：" << setting.split_count_ << endl;
		}
		return os;
	}
};

// 演算データ
// 数値を×mul_num＋add_numする役割を担う
struct Operation {
	// 掛け算する定数
	int mul_num = 1;
	// 足し算する定数
	int add_num = 0;
	// 文字列化
	string str() const noexcept {
		if (add_num == 0) {
			if (mul_num != 1) {
				return "*" + std::to_string(mul_num);
			}
			else {
				return "";
			}
		}
		else if (add_num > 0) {
			return "+" + std::to_string(add_num);
		}
		else {
			return "-" + std::to_string(std::abs(add_num));
		}
	}
	// 計算を適用
	inline int calc(const int x) const noexcept {
		return x * mul_num + add_num;
	}
};
// 方向データ
struct Direction {
	// 行き先
	size_t next_position;
	// 辺の番号
	size_t side_index;
};

// 問題データ
class Problem {
	// 頂点データ
	// field_[マス目][各方向] = 方向データ
	// [各方向]部分を可変長(vector)にしているのがポイント
	vector<vector<Direction>> field_;
	// 辺データ
	vector<Operation> side_;
	// 盤面サイズ
	size_t width_ = 0, height_ = 0;
	// スタート・ゴール
	size_t start_, goal_;
	// 既存の経路
	// 例えば<途中までの経路>が「1 0 8」だとスタート0・ゴール8・既存の経路「0」だが、
	// 「4 0 3 6 7 8」だとスタート7・ゴール8・既存の経路「0→3→6→7」になる
	vector<size_t> pre_root_;
	// 既存の得点
	// 起点における所持得点は1点だが、既存の経路に従って移動すると当然得点が変化する
	int pre_score_ = 1;
	// 地点A→地点Bに移動する際のインデックスを取得する
	// 取得できない場合は-1を返す
	int get_index(const size_t point_a, const size_t point_b)const noexcept{
		int result = -1;
		for (size_t i = 0; i < field_[point_a].size(); ++i) {
			if (field_[point_a][i].next_position == point_b) {
				result = static_cast<int>(i);
				break;
			}
		}
		return result;
	}
	void erase_root(const size_t point_a, const size_t point_b) {
		const int index_ab = get_index(point_a, point_b);
		const int index_ba = get_index(point_b, point_a);
		if(index_ab >= 0)
			field_[point_a].erase(field_[point_a].begin() + index_ab);
		if (index_ba >= 0)
		field_[point_b].erase(field_[point_b].begin() + index_ba);
	}
public:
	// コンストラクタ
	Problem(){}
	Problem(const string file_name, const int start_position, const int goal_position) {
		try {
			// ファイルを読み込む
			std::ifstream ifs(file_name);
			if (ifs.fail())
				throw "ファイルを読み込めません。";
			// 盤面サイズを読み込む
			{
				int width__, height__;
				ifs >> width__ >> height__;
				if (width__ < 1 || height__ < 1)
					throw "盤面サイズが間違っています。";
				width_ = width__;
				height_ = height__;
			}
			field_.resize(width_ * height_, vector<Direction>());
			start_ = (start_position >= 0 && start_position < width_ * height_ ? start_position : 0);
			goal_ = (goal_position >= 0 && goal_position < width_ * height_ ? goal_position : width_ * height_ - 1);
			// 盤面を読み込む
			for (size_t h = 0; h < height_ * 2 - 1; ++h) {
				for (size_t w = 0; w < (h % 2 == 0 ? width_ - 1 : width_); ++w) {
					if (ifs.eof())
						throw "盤面データが足りません。";
					// 一区切り(+1や-3や*7など)を読み込む
					string token;
					ifs >> token;
					// 2文字目以降を数字と認識する
					const int number = std::stoi(token.substr(1));
					// 演算子を読み取り、それによって場合分けを行う
					const string operation_str = token.substr(0, 1);
					Operation ope;
					if (operation_str == "+") {
						ope.add_num = number;
					}
					else if (operation_str == "-") {
						ope.add_num = -number;
					}
					else if (operation_str == "*") {
						ope.mul_num = number;
					}
					else {
						throw "不明な演算子です。";
					}
					// field_およびsideに代入する
					if (h % 2 == 0) {
						{
							// 横方向の経路─
							size_t x = w;
							size_t y = h / 2;
							size_t p = y * width_ + x;
							field_[p].push_back(Direction{ p + 1, side_.size() });
							field_[p + 1].push_back(Direction{ p, side_.size() });
						}
					}
					else {
						{
							// 縦方向の経路│
							size_t x = w;
							size_t y = (h - 1) / 2;
							size_t p = y * width_ + x;
							field_[p].push_back(Direction{ p + width_, side_.size() });
							field_[p + width_].push_back(Direction{ p, side_.size() });
						}
					}
					side_.push_back(ope);
				}
			}
			// スタート・移動経路・ゴールを読み込む
			if (ifs.eof()) {
				pre_root_.push_back(start_);
				return;
			}
			{
				int pre_root_size;
				ifs >> pre_root_size;
				if (pre_root_size < 0) {
					pre_root_.push_back(start_);
					return;
				}
				for (size_t i = 0; i < pre_root_size; ++i) {
					int pre_root_pos;
					ifs >> pre_root_pos;
					if (pre_root_pos < 0)
						throw "途中までの経路データが間違っています。";
					pre_root_.push_back(pre_root_pos);
				}
				int pre_root_goal;
				ifs >> pre_root_goal;
				if (pre_root_goal < 0)
					throw "途中までの経路データが間違っています。";
				start_ = pre_root_[pre_root_size - 1];
				goal_ = pre_root_goal;
			}
			// 読み取った移動経路に従い、問題を最適化
			if (pre_root_.size() > 1) {
				// 移動経路における演算を行い、同時にその演算子を削除
				for (size_t i = 0; i < pre_root_.size() - 1; ++i) {
					// 移動時の始点と終点を取得する
					size_t pos_src = pre_root_[i];
					size_t pos_dst = pre_root_[i + 1];
					const int index_sd = get_index(pos_src, pos_dst);
					// 演算子を利用した演算を行う
					const auto &ope = side_[field_[pos_src][index_sd].side_index];
					pre_score_ = ope.calc(pre_score_);
					// 演算に使用した部分を削除する
					erase_root(pos_src, pos_dst);
				}
				// 移動後に生じた「使用できない演算子」を削除して回る
				bool erease_flg;
				do {
					erease_flg = false;
					for (size_t y = 0; y < height_; ++y) {
						for (size_t x = 0; x < width_; ++x) {
							size_t pos_src = y * width_ + x;
							if (field_[pos_src].size() == 1 && pos_src != goal_) {
								size_t pos_dst = field_[pos_src][0].next_position;
								erase_root(pos_src, pos_dst);
								erease_flg = true;
								break;
							}
						}
						if (erease_flg)
							break;
					}
				} while (erease_flg);
			}
		}
		catch (const char *s) {
			throw s;
		}
		catch (...) {
			throw "問題ファイルとして解釈できませんでした。";
		}
	}
	// 辺の大きさを返す
	size_t side_size() const noexcept {
		return side_.size();
	}
	// 辺が使えるか否かを表すフラグ一覧(の初期値)を返す
	vector<char> get_side_flg() const {
		vector<char> side_flg(side_.size(), 0);
		for (const auto &point : field_) {
			for (const auto &dir : point) {
				side_flg[dir.side_index] = 1;
			}
		}
		return side_flg;
	}
	// 角にゴールがあるか？
	bool corner_goal_flg() const noexcept {
		return (goal_ == 0 || goal_ == width_ - 1 || goal_ == width_ * (height_ - 1) || goal_ == width_ * height_ - 1);
	}
	// getter
	size_t get_start() const noexcept {
		return start_;
	}
	size_t get_goal() const noexcept {
		return goal_;
	}
	const vector<Direction>& get_dir_list(const size_t point) const noexcept {
		return field_[point];
	}
	const Operation& get_operation(const size_t side_index) const noexcept {
		return side_[side_index];
	}
	int get_pre_score() const noexcept {
		return pre_score_;
	}
	// 出力用(等幅フォント用)
	friend ostream& operator << (ostream& os, const Problem& problem) {
		cout << "【問題】" << endl;
		cout << "盤面の規模：" << problem.width_ << "x" << problem.height_ << endl;
		cout << "初期得点：" << problem.pre_score_ << endl;
		// リッチな表示にするため、表示用の文字列配列を用意
		vector<vector<string>> output_board(problem.height_ * 2 + 1, vector<string>(problem.width_ * 2 + 1));
		// とりあえず枠線を割り当てる
		output_board[0][0] = "┌";
		output_board[0][problem.width_ * 2] = "┐";
		output_board[problem.height_ * 2][0] = "└";
		output_board[problem.height_ * 2][problem.width_ * 2] = "┘";
		for (size_t i = 0; i < problem.width_ - 1; ++i) {
			output_board[0][i * 2 + 2] = "┬";
			output_board[problem.height_ * 2][i * 2 + 2] = "┴";
		}
		for (size_t i = 0; i < problem.height_ - 1; ++i) {
			output_board[i * 2 + 2][0] = "├";
			output_board[i * 2 + 2][problem.width_ * 2] = "┤";
		}
		for (size_t j = 0; j < problem.height_ - 1; ++j) {
			for (size_t i = 0; i < problem.width_ - 1; ++i) {
				output_board[j * 2 + 2][i * 2 + 2] = "┼";
			}
		}
		for (size_t j = 0; j < problem.height_ + 1; ++j) {
			for (size_t i = 0; i < problem.width_; ++i) {
				output_board[j * 2][i * 2 + 1] = "─";
			}
		}
		for (size_t j = 0; j < problem.height_; ++j) {
			for (size_t i = 0; i < problem.width_ + 1; ++i) {
				output_board[j * 2 + 1][i * 2] = "│";
			}
		}
		for (size_t j = 0; j < problem.height_; ++j) {
			for (size_t i = 0; i < problem.width_; ++i) {
				output_board[j * 2 + 1][i * 2 + 1] = "　";
			}
		}
		// セル間の罫線を、演算子に置き換える
		for (size_t y = 0; y < problem.height_; ++y) {
			for (size_t x = 0; x < problem.width_; ++x) {
				size_t pos = y * problem.width_ + x;
				const auto &dir_list = problem.field_[pos];
				for (const auto &dir : dir_list) {
					const auto &side = problem.side_[dir.side_index];
					if (side.str() == "")
						continue;
					// 上
					if (pos == dir.next_position + problem.width_) {
						output_board[y * 2][x * 2 + 1] = side.str();
					}
					// 右
					if (pos + 1 == dir.next_position) {
						output_board[y * 2 + 1][x * 2 + 2] = side.str();
					}
					// 下
					if (pos + problem.width_ == dir.next_position) {
						output_board[y * 2 + 2][x * 2 + 1] = side.str();
					}
					// 左
					if (pos == dir.next_position + 1) {
						output_board[y * 2 + 1][x * 2] = side.str();
					}
				}
			}
		}
		// スタート・ゴールマークを入力する
		{
			// スタートマーク
			size_t sx = problem.start_ % problem.width_;
			size_t sy = problem.start_ / problem.width_;
			output_board[sy * 2 + 1][sx * 2 + 1] = "Ｓ";
			// ゴールマーク
			size_t gx = problem.goal_ % problem.width_;
			size_t gy = problem.goal_ / problem.width_;
			output_board[gy * 2 + 1][gx * 2 + 1] = "Ｇ";
		}
		// 途中経路を入力する
		{
			// 始点
			size_t rp = problem.pre_root_[0];
			size_t rx = rp % problem.width_;
			size_t ry = rp / problem.width_;
			if (output_board[ry * 2 + 1][rx * 2 + 1] == "　") {
				output_board[ry * 2 + 1][rx * 2 + 1] = "○";
			}
			// 途中経路
			size_t old_dir = 0;	//1から順に上・右・下・左であるものとする
			for (size_t i = 1; i < problem.pre_root_.size(); ++i) {
				// 上
				if (rp == problem.pre_root_[i] + problem.width_) {
					output_board[ry * 2][rx * 2 + 1] = "┃";
					if (i > 1) {
						switch (old_dir){
						case 1:	//上上
							output_board[ry * 2 + 1][rx * 2 + 1] = "┃";
							break;
						case 2:	//右上
							output_board[ry * 2 + 1][rx * 2 + 1] = "┛";
							break;
						case 4:	//左上
							output_board[ry * 2 + 1][rx * 2 + 1] = "┗";
							break;
						}
					}
					rp -= problem.width_;
					ry--;
					old_dir = 1;
				}
				// 右
				if (rp + 1 == problem.pre_root_[i]) {
					output_board[ry * 2 + 1][rx * 2 + 2] = "━";
					if (i > 1) {
						switch (old_dir) {
						case 1:	//上右
							output_board[ry * 2 + 1][rx * 2 + 1] = "┏";
							break;
						case 2:	//右右
							output_board[ry * 2 + 1][rx * 2 + 1] = "━";
							break;
						case 3:	//下右
							output_board[ry * 2 + 1][rx * 2 + 1] = "┗";
							break;
						}
					}
					rp += 1;
					rx++;
					old_dir = 2;
				}
				// 下
				if (rp + problem.width_ == problem.pre_root_[i]) {
					output_board[ry * 2 + 2][rx * 2 + 1] = "┃";
					if (i > 1) {
						switch (old_dir) {
						case 2:	//右下
							output_board[ry * 2 + 1][rx * 2 + 1] = "┓";
							break;
						case 3:	//下下
							output_board[ry * 2 + 1][rx * 2 + 1] = "┃";
							break;
						case 4:	//左下
							output_board[ry * 2 + 1][rx * 2 + 1] = "┏";
							break;
						}
					}
					rp += problem.width_;
					ry++;
					old_dir = 3;
				}
				// 左
				if (rp == problem.pre_root_[i] + 1) {
					output_board[ry * 2 + 1][rx * 2] = "━";
					if (i > 1) {
						switch (old_dir) {
						case 1:	//上左
							output_board[ry * 2 + 1][rx * 2 + 1] = "┓";
							break;
						case 3:	//下左
							output_board[ry * 2 + 1][rx * 2 + 1] = "┛";
							break;
						case 4:	//左左
							output_board[ry * 2 + 1][rx * 2 + 1] = "━";
							break;
						}
					}
					rp -= 1;
					rx--;
					old_dir = 4;
				}
			}
		}
		// 結果を文字列に変換する
		for (const auto &line : output_board) {
			for (const auto &str : line) {
				os << str;
			}
			os << endl;
		}
		return os;
	}
};

// 解答データ
class Result {
	vector<size_t> root_;
	size_t ptr_;
	int score_;
public:
	// コンストラクタ
	Result(){}
	Result(const size_t side_size, const size_t start, const int score) : ptr_(0) {
		root_.resize(side_size);
		root_[ptr_] = start;
		score_ = score;
	}
	// 辺を移動した際の操作
	void move_side(const size_t point) noexcept {
		++ptr_;
		root_[ptr_] = point;
	}
	// 辺を戻した際の操作
	void back_side() noexcept {
		--ptr_;
	}
	// 現在の位置
	size_t now_point() const noexcept {
		return root_[ptr_];
	}
	// getterとsetter
	int get_score() const noexcept {
		return score_;
	}
	void set_score(const int score) noexcept {
		score_ = score;
	}
	// 出力用(等幅フォント用)
	friend ostream& operator << (ostream& os, const Result& result) {
		for (size_t i = 0; i <= result.ptr_; ++i) {
			if (i != 0)
				os << "->";
			os << result.root_[i];
		}
		return os;
	}
};

// ソルバー
class Solver {
	Problem problem_;
	Result result_, best_result_;
	vector<char> side_flg_;

	// 普通の深さ優先探索を行う
	Result dfs(const Problem &problem, const bool corner_goal_flg) {
		problem_ = problem;
		// 探索の起点となる解・最適解
		best_result_ = result_ = Result(problem.side_size(), problem.get_start(), problem.get_pre_score());
		best_result_.set_score(-9999);
		// ある辺を踏破したか？
		side_flg_ = problem.get_side_flg();
		// 探索開始
		if (corner_goal_flg) {
			dfs_cg();
		}
		else {
			dfs(problem_, result_, best_result_, side_flg_);
		}
		return best_result_;
	}
	void dfs_cg() noexcept {
		// ゴール地点なら、とりあえずスコア判定を行う
		if (result_.now_point() == problem_.get_goal()) {
			if (result_.get_score() > best_result_.get_score()) {
				best_result_ = result_;
				//cout << best_result.get_score() << "," << best_result << endl;
			}
			return;
		}
		// ネストを深くする
		for (const auto &dir : problem_.get_dir_list(result_.now_point())) {
			if (!side_flg_[dir.side_index])
				continue;
			// 進める
			const int old_score = result_.get_score();
			result_.move_side(dir.next_position);
			result_.set_score(problem_.get_operation(dir.side_index).calc(result_.get_score()));
			side_flg_[dir.side_index] = 0;
			// 再帰を一段階深くする
			dfs_cg();
			// 戻す
			side_flg_[dir.side_index] = 1;
			result_.back_side();
			result_.set_score(old_score);
		}
	}
	void dfs(
		const Problem &problem,	//問題データ
		Result &result,			//現在の進行ルート
		Result &best_result,	//最適解
		vector<char> &side_flg	//ある辺を踏破したか
	) noexcept {
		// ゴール地点なら、とりあえずスコア判定を行う
		if (result.now_point() == problem.get_goal()) {
			if (result.get_score() > best_result.get_score()) {
				best_result = result;
				//cout << best_result.get_score() << "," << best_result << endl;
			}
		}
		// ネストを深くする
		for (const auto &dir : problem.get_dir_list(result.now_point())) {
			if (!side_flg[dir.side_index])
				continue;
			// 進める
			const int old_score = result.get_score();
			result.move_side(dir.next_position);
			result.set_score(problem.get_operation(dir.side_index).calc(result.get_score()));
			side_flg[dir.side_index] = 0;
			// 再帰を一段階深くする
			dfs(problem, result, best_result, side_flg);
			// 戻す
			side_flg[dir.side_index] = 1;
			result.back_side();
			result.set_score(old_score);
		}
	}
public:
	// コンストラクタ
	Solver() {}
	// 解を探索する
	Result solve(const Problem &problem, unsigned int threads) {
		return dfs(problem, problem.corner_goal_flg());
	}
	// 問題を分割保存する
	void split(const Problem &problem, unsigned int splits) const {

	}
};

int main(int argc, char* argv[]) {
	try {
		// コマンドライン引数から、ソフトウェアの動作設定を読み取る
		Setting setting(argc, argv);
		//cout << setting << endl;
		// 問題ファイルを読み取る
		Problem problem(setting.file_name(), setting.start_position(), setting.goal_position());
		//cout << problem << endl;
		//
		Solver solver;
		if (setting.solver_flg()) {
			// 解を探索する
			for (size_t i = 0; i < 20; ++i) {
				StopWatch sw;
				sw.Start();
				const auto result = solver.solve(problem, setting.split_count());
				sw.Stop();
				cout << sw.ElapsedMilliseconds() << endl;
			}
			// 結果を表示する
			//cout << "【結果】" << endl;
			//cout << "最高得点：" << result.get_score() << endl;
			//cout << "最高経路：" << result << endl;
			//cout << "計算時間：" << sw.ElapsedMilliseconds() << "[ms]" << endl;
			return 0;
		}
		else {
			// 問題を分割保存する
		}
	}
	catch (const char *s) {
		cout << "エラー：" << s << endl;
		return EXIT_FAILURE;
	}
	catch(...){
		return EXIT_FAILURE;
	}
}

/* このコミットの記録(2484.1±平均誤差2.85703)
2492
2509
2484
2490
2490
2489
2467
2468
2472
2488
2472
2471
2470
2505
2473
2482
2476
2496
2486
2502
*/
