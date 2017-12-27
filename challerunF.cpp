#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>
#include <mutex>
#include "ThreadPool.h"

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
	void Start() { start_time = std::chrono::system_clock::now(); }
	// ストップウォッチを停止
	void Stop() { end_time = std::chrono::system_clock::now(); }
	// 経過時間を返す
	long long ElapsedNanoseconds() const {
		return std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
	}
	long long ElapsedMicroseconds() const {
		return std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
	}
	long long ElapsedMilliseconds() const {
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
	string file_name() const { return file_name_; }
	int start_position() const { return start_position_; }
	int goal_position() const { return goal_position_; }
	bool solver_flg() const { return solver_flg_; }
	unsigned int split_count() const { return split_count_; }
	// 出力用
	friend ostream& operator << (ostream& os, const Setting& setting) {
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
	int add_num_x = 0;	//add_numが1以上ならadd_num、0以下なら0とする
	// 文字列化
	string str() const {
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
	inline int calc(const int x) const {
		return x * mul_num + add_num;
	}
	// 2つのOperationを合体させる(非可換)
	// 「Operation1 + Operation2」を、「Operation1の後にOperation2を適用する」といった意味とする
	const Operation operator + (const Operation& b) const {
		// Operation1をM1*X+A1、Operation2をM2*X+A2とする。
		// Operation1→Operation2の順で適用すると、
		// M2*(M1*X+A1)+A2=M1*M2*X+A1*M2+A2 となる
		const int mul_num2 = this->mul_num * b.mul_num;
		const int add_num2 = this->add_num * b.mul_num + b.add_num;
		return Operation{ mul_num2, add_num2, (add_num2 > 0 ? add_num2 : 0) };
	}
};
// 方向データ
struct Direction {
	// 行き先
	size_t next_position;
	// 辺の番号
	size_t side_index;
	// 辺の情報
	Operation operation;
};
struct Direction2 {
	// 行き先
	size_t next_position1, next_position2;
	// 辺の番号
	size_t side_index1, side_index2;
	// 辺の情報
	Operation operation;
};

// 問題データ
class Problem {
	// 頂点データ
	// field_[マス目][各方向] = 方向データ
	// [各方向]部分を可変長(vector)にしているのがポイント
	vector<vector<Direction>> field_;
	//頂点データ(2歩編)
	vector<vector<Direction2>> field2_;
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
	int get_index(const size_t point_a, const size_t point_b)const {
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
						ope.add_num_x = std::max(0, number);
					}
					else if (operation_str == "-") {
						ope.add_num = -number;
						ope.add_num_x = std::max(0, -number);
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
							field_[p].push_back(Direction{ p + 1, side_.size(), ope });
							field_[p + 1].push_back(Direction{ p, side_.size(), ope });
						}
					}
					else {
						{
							// 縦方向の経路│
							size_t x = w;
							size_t y = (h - 1) / 2;
							size_t p = y * width_ + x;
							field_[p].push_back(Direction{ p + width_, side_.size(), ope });
							field_[p + width_].push_back(Direction{ p, side_.size(), ope });
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
			// field2_を作成する
			field2_.resize(width_ * height_, vector<Direction2>());
			for (size_t p = 0; p < width_ * height_; ++p) {
				for (const auto &next1 : field_[p]) {
					for (const auto &next2 : field_[next1.next_position]) {
						if (next2.next_position == p)
							continue;
						const auto &next_position1 = next1.next_position;
						const auto &next_position2 = next2.next_position;
						const auto &side_index1 = next1.side_index;
						const auto &side_index2 = next2.side_index;
						const Operation operation = next1.operation + next2.operation;
						Direction2 dir2 = { next_position1, next_position2 , side_index1, side_index2, operation };
						field2_[p].push_back(dir2);
					}
				}
			}
			return;
		}
		catch (const char *s) {
			throw s;
		}
		catch (...) {
			throw "問題ファイルとして解釈できませんでした。";
		}
	}
	// 辺の大きさを返す
	size_t side_size() const {
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
		for (size_t i = 0; i < pre_root_.size() - 1; ++i) {
			const auto index = get_index(pre_root_[i], pre_root_[i + 1]);
			if(index >= 0)
				side_flg[field_[pre_root_[i]][index].side_index] = 0;
		}
		return side_flg;
	}
	// ある地点の周りにある、まだ通れる辺の数の初期値を返す
	// (ただしゴール地点だけ+1しておく)
	vector<char> get_available_side_count() const {
		vector<char> available_side_count(width_ * height_, 0);
		for (size_t i = 0; i < field_.size(); ++i) {
			available_side_count[i] = field_[i].size();
		}
		available_side_count[goal_] += 1;
		return available_side_count;
	}
	// 角にゴールがあるか？
	bool corner_goal_flg() const {
		return (goal_ == 0 || goal_ == width_ - 1 || goal_ == width_ * (height_ - 1) || goal_ == width_ * height_ - 1);
	}
	// 問題の奇偶を調べる
	bool is_odd() const {
		int sx = start_ % width_, sy = start_ / width_;
		int gx = goal_ % width_, gy = goal_ / width_;
		int x_flg = std::abs(sx - gx) % 2, y_flg = std::abs(sy - gy) % 2;
		return ((x_flg + y_flg) % 2 == 1);
	}
	// 移動操作
	void move(const size_t next_position) {
		pre_root_.push_back(next_position);
		pre_score_ = side_[field_[start_][get_index(start_, next_position)].side_index].calc(pre_score_);
		start_ = next_position;
	}
	// 保存用に書き出す
	string to_file() const {
		std::ostringstream oss;
		oss << width_ << " " << height_ << endl;
		size_t p = 0;
		for (size_t h = 0; h < height_ * 2 - 1; ++h) {
			for (size_t w = 0; w < (h % 2 == 0 ? width_ - 1 : width_); ++w) {
				if (w != 0) oss << " ";
				oss << side_[p].str();
				++p;
			}
			oss << endl;
		}
		oss << pre_root_.size() << " ";
		for (size_t i = 0; i < pre_root_.size(); ++i) {
			oss << pre_root_[i] << " ";
		}
		oss << goal_ << endl;
		return oss.str();
	}
	// 獲得可能な得点の上限を算出するための数値
	void get_muladd_value(const vector<char> &side_flg, int &max_mul_value, int &max_add_value) const {
		// 初期値
		max_mul_value = 1; max_add_value = 0;
		// 各辺についてチェックする
		for (size_t i = 0; i < side_.size(); ++i) {
			if (!side_flg[i])
				continue;
			if (side_[i].add_num != 0) {
				// 加減算
				max_add_value += std::max(0, side_[i].add_num);
			}
			else if(side_[i].mul_num != 0){
				// 乗算
				max_mul_value *= std::max(1, side_[i].mul_num);
			}
		}
	}
	// getter
	size_t get_start() const {
		return start_;
	}
	size_t get_goal() const {
		return goal_;
	}
	size_t get_width() const {
		return width_;
	}
	size_t get_height() const {
		return height_;
	}
	const vector<size_t> &get_pre_root() const {
		return pre_root_;
	}
	const vector<Direction>& get_dir_list(const size_t point) const {
		return field_[point];
	}
	const vector<Direction2>& get_dir_list2(const size_t point) const {
		return field2_[point];
	}
	const Operation& get_operation(const size_t side_index) const {
		return side_[side_index];
	}
	int get_pre_score() const {
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
public:
	// コンストラクタ
	Result(){}
	Result(const size_t side_size, const size_t start) : ptr_(0) {
		root_.resize(side_size);
		root_[ptr_] = start;
	}
	// 辺を移動した際の操作
	void move_side(const size_t point) {
		++ptr_;
		root_[ptr_] = point;
	}
	// 辺を戻した際の操作
	void back_side() {
		--ptr_;
	}
	void back_side2() {
		ptr_ -= 2;
	}
	// 現在の位置
	size_t now_position() const {
		return root_[ptr_];
	}
	// getter
	vector<size_t> get_root() const {
		return vector<size_t>(root_.begin(), root_.begin() + ptr_ + 1);
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
size_t g_threads = 1;
size_t g_max_threads;
std::mutex mtx;
int g_best_score = -9999;
class Solver {
	Problem problem_;
	Result result_, best_result_;
	int score_, best_score_;
	vector<char> side_flg_;
	vector<char> available_side_count_;
	int max_mul_value_, max_add_value_;

	// 普通の深さ優先探索を行う
	std::pair<Result, int> dfs(const Problem &problem, const bool corner_goal_flg) {
		problem_ = problem;
		// 探索の起点となる解・最適解
		best_result_ = result_ = Result(problem.side_size(), problem.get_start());
		score_ = problem.get_pre_score();
		best_score_ = -9999;
		// ある辺を踏破したか？
		side_flg_ = problem.get_side_flg();
		// ある地点の周りにある、まだ通れる辺の数
		// (ただしゴール地点だけ+1しておく)
		available_side_count_ = problem.get_available_side_count();
		// 獲得可能な得点の上限を算出するための数値
		problem.get_muladd_value(side_flg_, max_mul_value_, max_add_value_);
		// 探索開始
		if (corner_goal_flg) {
			// 始点と終点の奇偶を調べる
			if (problem.is_odd()) {
				dfs_cg_b(result_.now_position());
			}
			else {
				dfs_cg_a(result_.now_position());
			}
		}
		else {
			// 始点と終点の奇偶を調べる
			if (problem.is_odd()) {
				dfs_b(result_.now_position());
			}
			else {
				dfs_a(result_.now_position());
			}
		}
		Result best_result2(problem.side_size(), problem.get_pre_root()[0]);
		for (size_t i = 1; i < problem.get_pre_root().size() - 1; ++i) {
			best_result2.move_side(problem.get_pre_root()[i]);
		}
		for (size_t i = 0; i < best_result_.get_root().size(); ++i) {
			best_result2.move_side(best_result_.get_root()[i]);
		}
		return std::pair<Result, int>(best_result2, best_score_);
	}
	void dfs_cg_a(const size_t now_position) {
		// ゴール地点なら、とりあえずスコア判定を行う
		if (now_position == problem_.get_goal()) {
			if (score_ > best_score_) {
				best_result_ = result_;
				best_score_ = score_;
				mtx.lock();
				g_best_score = best_score_;
				mtx.unlock();
				//cout << best_result_.get_score() << "," << best_result_ << endl;
			}
			return;
		}
		// 見込みスコアが現時点のベストスコアに劣っている場合は戻る
		if ((score_ + max_add_value_) * max_mul_value_ < g_best_score)
			return;
		// ネストを深くする
		--available_side_count_[now_position];
		for (const auto &dir : problem_.get_dir_list2(now_position)) {
			if (!side_flg_[dir.side_index1] || !side_flg_[dir.side_index2])
				continue;
			if (available_side_count_[dir.next_position2] <= 1)
				continue;
			// 進める
			const int old_score = score_;
			--available_side_count_[dir.next_position2];
			result_.move_side(dir.next_position1);
			result_.move_side(dir.next_position2);
			score_ = dir.operation.calc(score_);
			side_flg_[dir.side_index1] = 0;
			side_flg_[dir.side_index2] = 0;
			max_mul_value_ /= problem_.get_operation(dir.side_index1).mul_num;
			max_mul_value_ /= problem_.get_operation(dir.side_index2).mul_num;
			max_add_value_ -= problem_.get_operation(dir.side_index1).add_num_x;
			max_add_value_ -= problem_.get_operation(dir.side_index2).add_num_x;
			// 再帰を一段階深くする
			dfs_cg_a(dir.next_position2);
			// 戻す
			side_flg_[dir.side_index1] = 1;
			side_flg_[dir.side_index2] = 1;
			max_mul_value_ *= problem_.get_operation(dir.side_index1).mul_num;
			max_mul_value_ *= problem_.get_operation(dir.side_index2).mul_num;
			max_add_value_ += problem_.get_operation(dir.side_index1).add_num_x;
			max_add_value_ += problem_.get_operation(dir.side_index2).add_num_x;
			result_.back_side2();
			++available_side_count_[dir.next_position2];
			score_ = old_score;
		}
		++available_side_count_[now_position];
	}
	void dfs_cg_b(const size_t now_position) {
		// ゴール地点なら、とりあえずスコア判定を行う
		if (now_position == problem_.get_goal()) {
			if (score_ > best_score_) {
				best_result_ = result_;
				best_score_ = score_;
				mtx.lock();
				g_best_score = best_score_;
				mtx.unlock();
			}
			return;
		}
		// 見込みスコアが現時点のベストスコアに劣っている場合は戻る
		if ((score_ + max_add_value_) * max_mul_value_ < g_best_score)
			return;
		// ネストを深くする
		--available_side_count_[now_position];
		for (const auto &dir : problem_.get_dir_list(now_position)) {
			if (!side_flg_[dir.side_index])
				continue;
			if (available_side_count_[dir.next_position] <= 1)
				continue;
			// 進める
			const int old_score = score_;
			--available_side_count_[dir.next_position];
			result_.move_side(dir.next_position);
			score_ = problem_.get_operation(dir.side_index).calc(score_);
			side_flg_[dir.side_index] = 0;
			max_mul_value_ /= problem_.get_operation(dir.side_index).mul_num;
			max_add_value_ -= problem_.get_operation(dir.side_index).add_num_x;
			// 再帰を一段階深くする
			dfs_cg_a(dir.next_position);
			// 戻す
			side_flg_[dir.side_index] = 1;
			max_mul_value_ *= problem_.get_operation(dir.side_index).mul_num;
			max_add_value_ += problem_.get_operation(dir.side_index).add_num_x;
			result_.back_side();
			++available_side_count_[dir.next_position];
			score_ = old_score;
		}
		++available_side_count_[now_position];
	}
	void dfs_a(const size_t now_position) {
		// ゴール地点なら、とりあえずスコア判定を行う
		if (now_position == problem_.get_goal()) {
			if (score_ > best_score_) {
				best_result_ = result_;
				best_score_ = score_;
				mtx.lock();
				g_best_score = best_score_;
				mtx.unlock();
				//cout << best_result_.get_score() << "," << best_result_ << endl;
			}
		}
		// 見込みスコアが現時点のベストスコアに劣っている場合は戻る
		if ((score_ + max_add_value_) * max_mul_value_ < g_best_score)
			return;
		// ネストを深くする
		--available_side_count_[now_position];
		for (const auto &dir : problem_.get_dir_list2(now_position)) {
			if (!side_flg_[dir.side_index1] || !side_flg_[dir.side_index2])
				continue;
			if (available_side_count_[dir.next_position2] <= 1)
				continue;
			// 進める
			const int old_score = score_;
			--available_side_count_[dir.next_position2];
			result_.move_side(dir.next_position1);
			result_.move_side(dir.next_position2);
			score_ = dir.operation.calc(score_);
			side_flg_[dir.side_index1] = 0;
			side_flg_[dir.side_index2] = 0;
			max_mul_value_ /= problem_.get_operation(dir.side_index1).mul_num;
			max_mul_value_ /= problem_.get_operation(dir.side_index2).mul_num;
			max_add_value_ -= problem_.get_operation(dir.side_index1).add_num_x;
			max_add_value_ -= problem_.get_operation(dir.side_index2).add_num_x;
			// 再帰を一段階深くする
			dfs_a(dir.next_position2);
			// 戻す
			side_flg_[dir.side_index1] = 1;
			side_flg_[dir.side_index2] = 1;
			max_mul_value_ *= problem_.get_operation(dir.side_index1).mul_num;
			max_mul_value_ *= problem_.get_operation(dir.side_index2).mul_num;
			max_add_value_ += problem_.get_operation(dir.side_index1).add_num_x;
			max_add_value_ += problem_.get_operation(dir.side_index2).add_num_x;
			result_.back_side2();
			++available_side_count_[dir.next_position2];
			score_ = old_score;
		}
		++available_side_count_[now_position];
	}
	void dfs_b(const size_t now_position) {
		// ゴール地点なら、とりあえずスコア判定を行う
		if (now_position == problem_.get_goal()) {
			if (score_ > best_score_) {
				best_result_ = result_;
				best_score_ = score_;
				mtx.lock();
				g_best_score = best_score_;
				mtx.unlock();
			}
			return;
		}
		// 見込みスコアが現時点のベストスコアに劣っている場合は戻る
		if ((score_ + max_add_value_) * max_mul_value_ < g_best_score)
			return;
		// ネストを深くする
		--available_side_count_[now_position];
		for (const auto &dir : problem_.get_dir_list(now_position)) {
			if (!side_flg_[dir.side_index])
				continue;
			if (available_side_count_[dir.next_position] <= 1)
				continue;
			// 進める
			const int old_score = score_;
			--available_side_count_[dir.next_position];
			result_.move_side(dir.next_position);
			score_ = problem_.get_operation(dir.side_index).calc(score_);
			side_flg_[dir.side_index] = 0;
			max_mul_value_ /= problem_.get_operation(dir.side_index).mul_num;
			max_add_value_ -= problem_.get_operation(dir.side_index).add_num_x;
			// 再帰を一段階深くする
			dfs_a(dir.next_position);
			// 戻す
			side_flg_[dir.side_index] = 1;
			max_mul_value_ *= problem_.get_operation(dir.side_index).mul_num;
			max_add_value_ += problem_.get_operation(dir.side_index).add_num_x;
			result_.back_side();
			++available_side_count_[dir.next_position];
			score_ = old_score;
		}
		++available_side_count_[now_position];
	}
public:
	// コンストラクタ
	Solver() {}
	// 解を探索する
	std::pair<Result, int> solve(const Problem &problem, unsigned int threads) {
		if (threads == 1) {
			return dfs(problem, problem.corner_goal_flg());
		}
		problem_ = problem;
		// 探索の起点となる解・最適解
		best_result_ = result_ = Result(problem.side_size(), problem.get_start());
		score_ = problem.get_pre_score();
		best_score_ = -9999;
		// ある辺を踏破したか？
		side_flg_ = problem.get_side_flg();
		// ある地点の周りにある、まだ通れる辺の数
		// (ただしゴール地点だけ+1しておく)
		available_side_count_ = problem.get_available_side_count();
		// 始点
		// 探索開始
		const auto problem_list = split(problem, threads * 100);
		vector<std::future<std::pair<Result, int>>> result_list_future;
		ThreadPool pool(threads);
		for (const auto &problem_temp : problem_list) {
			result_list_future.emplace_back(
				pool.enqueue([&] {
				Solver new_solver;
				return new_solver.dfs(problem_temp, problem_temp.corner_goal_flg());
			})
			);
		}
		vector<std::pair<Result, int>> result_list;
		for (auto && result : result_list_future) {
			result_list.emplace_back(result.get());
		}
		best_score_ = -9999;
		for (size_t di = 0; di < result_list.size(); ++di) {
			if (best_score_ < result_list[di].second) {
				best_result_ = result_list[di].first;
				best_score_ = result_list[di].second;
			}
		}
		return std::pair<Result, int>(best_result_, best_score_);
	}
	// 問題を分割保存する
	vector<Problem> split(const Problem &problem) const {
		vector<Problem> splited_problem;
		const auto side_flg = problem.get_side_flg();
		for (const auto &dir : problem.get_dir_list(problem.get_start())) {
			if (!side_flg[dir.side_index])
				continue;
			Problem next_problem = problem;
			next_problem.move(dir.next_position);
			splited_problem.push_back(next_problem);
		}
		return splited_problem;
	}
	vector<Problem> split(const Problem &problem, unsigned int splits) const {
		vector<Problem> splited_problem;
		splited_problem.push_back(problem);
		// ・splited_problemの各問題について、1段階分割した後にsplited_problem2に追記する
		// ・splited_problemをsplited_problem2で上書きする
		// ・上1つを続けると、nステップ目にsplited_problemの要素数がsplits以上になるのでループを抜ける
		do {
			vector<Problem> splited_problem2;
			for (size_t i = 0; i < splited_problem.size(); ++i) {
				const auto temp = split(splited_problem[i]);
				for (const auto &q : temp) {
					splited_problem2.push_back(q);
				}
				if (splited_problem2.size() + splited_problem.size() - i - 1 >= splits) {
					for (size_t j = i + 1; j < splited_problem.size(); ++j) {
						splited_problem2.push_back(splited_problem[j]);
					}
					break;
				}
			}
			splited_problem.clear();
			splited_problem = splited_problem2;

		}while (splited_problem.size() < splits);
		return splited_problem;
	}
};

int main(int argc, char* argv[]) {
	try {
		// コマンドライン引数から、ソフトウェアの動作設定を読み取る
		Setting setting(argc, argv);
		// 問題ファイルを読み取る
		Problem problem(setting.file_name(), setting.start_position(), setting.goal_position());
		//
		if (setting.solver_flg()) {
			// 解を探索する
			{
				Solver solver;
				StopWatch sw;
				sw.Start();
				std::pair<Result, int> result = solver.solve(problem, setting.split_count());
				sw.Stop();
				cout << problem.get_width() << "," << problem.get_height() << "," << result.second << "," << result.first << "," << (1.0 * sw.ElapsedMilliseconds() / 1000) << endl;
			}
			return 0;
		}
		else {
			// 問題を分割保存する
			// まず分割する
			Solver solver;
			const auto splited_problem = solver.split(problem, setting.split_count());
			// ファイル保存のための準備をする
			string file_name_without_ext;
			string::size_type pos = setting.file_name().find_last_of(".");
			if (pos == string::npos) {
				file_name_without_ext = setting.file_name();
			}
			else {
				file_name_without_ext = setting.file_name().substr(0, pos);
			}
			// 保存処理
			for (size_t i = 0; i < splited_problem.size(); ++i) {
				int zero_count = std::to_string(splited_problem.size()).size() - std::to_string(i + 1).size();
				const auto hoge = file_name_without_ext + "_" + string(zero_count, '0') + std::to_string(i + 1) + ".txt";
				std::ofstream ofs(file_name_without_ext + "_" + string(zero_count, '0') + std::to_string(i + 1) + ".txt");
				ofs << splited_problem[i].to_file();
			}
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
