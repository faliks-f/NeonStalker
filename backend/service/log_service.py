import os
import json
from datetime import datetime, timedelta


class LogService:
    LOG_DIR = "logs"

    def __init__(self):
        os.makedirs(self.LOG_DIR, exist_ok=True)

    def _get_log_file(self):
        """返回当天的日志文件路径"""
        date_str = datetime.now().strftime("%Y-%m-%d")
        return os.path.join(self.LOG_DIR, f"{date_str}.log")

    def write_log(self, role, action, level="INFO", module="All"):
        """
        写入日志
        :param role: 角色 (user/admin)
        :param action: 操作 (开始直播、结束直播等)
        :param level: 日志级别 (INFO, WARNING, ERROR)
        :param module: 模块 (All, Camera, Car, Arm)
        """
        log_data = {
            "role": role,
            "action": action,
            "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "level": level,
            "module": module
        }
        log_file = self._get_log_file()
        with open(log_file, "a", encoding="utf-8") as f:
            f.write(json.dumps(log_data, ensure_ascii=False) + "\n")

    def query(self):
        """返回一个 LogQuery 实例，支持链式查询"""
        return LogQuery(self.LOG_DIR)


class LogQuery:
    def __init__(self, log_dir):
        self.log_dir = log_dir
        self.filters = []
        self.start_date = None
        self.end_date = None

    def filter_by_role(self, role):
        self.filters.append(lambda log: log["role"] == role)
        return self

    def filter_by_action(self, action):
        self.filters.append(lambda log: log["action"] == action)
        return self

    def filter_by_level(self, level):
        self.filters.append(lambda log: log["level"] == level)
        return self

    def filter_by_module(self, module):
        self.filters.append(lambda log: log["module"] == module)
        return self

    def filter_by_date_range(self, start_date, end_date):
        self.start_date = start_date
        self.end_date = end_date
        return self

    def execute(self):
        """执行查询并返回匹配的日志"""
        results = []
        log_files = self._get_log_files()

        for log_file in log_files:
            with open(log_file, "r", encoding="utf-8") as f:
                for line in f:
                    try:
                        log = json.loads(line.strip())
                        if self._apply_filters(log):
                            results.append(log)
                    except json.JSONDecodeError:
                        continue
        return results

    def _apply_filters(self, log):
        """应用所有筛选条件"""
        if self.start_date and self.end_date:
            log_time = datetime.strptime(log["time"], "%Y-%m-%d %H:%M:%S")
            if not (self.start_date <= log_time <= self.end_date):
                return False
        return all(f(log) for f in self.filters)

    def _get_log_files(self):
        """获取指定时间范围内的日志文件"""
        log_files = []
        if self.start_date and self.end_date:
            current_date = self.start_date
            while current_date <= self.end_date:
                log_file = os.path.join(self.log_dir, f"{current_date.strftime('%Y-%m-%d')}.log")
                if os.path.exists(log_file):
                    log_files.append(log_file)
                current_date += timedelta(days=1)
        else:
            # 读取所有日志文件
            for filename in os.listdir(self.log_dir):
                if filename.endswith(".log"):
                    log_files.append(os.path.join(self.log_dir, filename))
        return log_files

log_service = LogService()

# 示例调用
if __name__ == "__main__":
    log_service = LogService()

    # 写入日志
    log_service.write_log("user", "开始直播")
    log_service.write_log("admin", "结束直播", level="WARNING", module="Camera")

    # 进行链式查询
    logs = (
        log_service.query()
        .filter_by_role("user")
        .filter_by_action("开始直播")
        .filter_by_date_range(datetime(2025, 3, 1), datetime(2025, 3, 18))
        .execute()
    )

    for log in logs:
        print(log)
