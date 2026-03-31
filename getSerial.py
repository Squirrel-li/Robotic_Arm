import argparse
import serial
import sys

# 預設欄位格式：如果標題名稱匹配，則使用這些寬度/小數位數
# 如果您有新的欄位，可以添加到這裡以獲得更好的格式化
DEFAULT_COLUMN_FORMATS = {
    "currentAngle.x": (9, 3),
    "currentAngle.y": (9, 3),
    "currentAngle.z": (9, 3),
}

# 未知欄位的預設設定
FALLBACK_WIDTH = 10
FALLBACK_DECIMALS = 3

# 全域變數，用於儲存當前活躍的欄位定義
_active_columns_def = None # 格式為 [(名稱, 寬度, 小數位數), ...]
_header_printed = False    # 標誌，確保每個標題定義只列印一次


def format_header(columns_def) -> str:
    """根據給定的欄位定義格式化標題行"""
    if not columns_def:
        return ""
    return " ".join([f"{name:>{width}}" for name, width, _ in columns_def])


def format_row(columns_def, values) -> str:
    """根據給定的欄位定義和數值格式化數據行"""
    if not columns_def or not values or len(columns_def) != len(values):
        return ""
    out = []
    for (_, width, decimals), v in zip(columns_def, values):
        out.append(f"{v:>{width}.{decimals}f}")
    return " ".join(out)


def parse_csv_line_to_floats(line: str, expected_col_count: int):
    """嘗試將 CSV 行解析為浮點數列表"""
    parts = [p.strip() for p in line.split(",")]
    if len(parts) != expected_col_count:
        return None
    try:
        return [float(x) for x in parts]
    except ValueError:
        return None


def main():
    parser = argparse.ArgumentParser(description="從序列埠讀取 CSV 並列印固定寬度的表格。")
    parser.add_argument("--port", required=True, help="序列埠，例如 COM5")
    parser.add_argument("--baud", type=int, default=115200, help="鮑率")
    parser.add_argument("--debug", action="store_true", help="如果解析失敗，則列印原始行")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except Exception as e:
        print(f"開啟序列埠失敗: {e}", file=sys.stderr)
        sys.exit(1)

    global _active_columns_def, _header_printed

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            # 判斷是否為潛在的標題行：
            # - 包含逗號
            # - 拆分後有多個部分
            # - 嘗試解析為浮點數會失敗（表示不是數據行，可能是標題）
            is_potential_header = "," in line and len(line.split(',')) > 1
            if is_potential_header:
                try:
                    # 嘗試將其解析為數據行。如果成功，則它不是標題。
                    temp_parts = [p.strip() for p in line.split(',')]
                    [float(x) for x in temp_parts]
                    is_potential_header = False # 這實際上是一行數據
                except ValueError:
                    # 解析為浮點數失敗，因此它可能是標題。
                    is_potential_header = True

            if is_potential_header:
                new_col_names = [p.strip() for p in line.split(",")]
                
                # 根據名稱建構新的欄位定義
                current_col_defs = []
                for name in new_col_names:
                    if name in DEFAULT_COLUMN_FORMATS:
                        width, decimals = DEFAULT_COLUMN_FORMATS[name]
                    else:
                        # 未知欄位的預設值
                        width, decimals = FALLBACK_WIDTH, FALLBACK_DECIMALS
                    current_col_defs.append((name, width, decimals))

                # 檢查標題定義是否已更改以避免重複列印
                header_has_changed = False
                if _active_columns_def is None or \
                   len(_active_columns_def) != len(current_col_defs) or \
                   any(old[0] != new[0] for old, new in zip(_active_columns_def, current_col_defs)):
                    header_has_changed = True
                
                _active_columns_def = current_col_defs
                
                if header_has_changed or not _header_printed: # 如果更改或尚未列印，則強制重新列印
                    # 清除前一行（如果有）並列印新標題
                    sys.stdout.write("\r" + " " * (len(format_header(_active_columns_def) if _active_columns_def else "") + 5) + "\r")
                    print(format_header(_active_columns_def))
                    print("-" * len(format_header(_active_columns_def)))
                    _header_printed = True
                
                # 不要將此行作為數據處理
                continue

            # 處理狀態訊息（不清除行）
            if "ready" in line.lower() or "failed" in line.lower() or "init" in line.lower() or "scan" in line.lower():
                # 清除前一行（如果有）並列印狀態訊息
                sys.stdout.write("\r" + " " * (len(format_header(_active_columns_def) if _active_columns_def else "") + 5) + "\r")
                print(line)
                continue
            
            # 如果尚未收到標題，則跳過數據行
            if _active_columns_def is None:
                if args.debug:
                    print(f"\n[RAW] 尚未定義標題: {line}")
                continue

            # 如果標題定義可用，則嘗試解析數據
            values = parse_csv_line_to_floats(line, len(_active_columns_def))
            if values is None:
                if args.debug:
                    print(f"\n[RAW] 解析失敗: {line}")
                continue

            # 列印數據行，覆蓋當前行
            row_output = format_row(_active_columns_def, values)
            sys.stdout.write("\r" + row_output)
            sys.stdout.flush()

    except KeyboardInterrupt:
        print("\n程式結束。")
    except Exception as e:
        print(f"\n發生錯誤: {e}", file=sys.stderr)
    finally:
        if ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()