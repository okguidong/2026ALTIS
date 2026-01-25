import tkinter as tk
from tkinter import filedialog, messagebox
import ctypes
import os
import pandas as pd

# --- [1] 데이터 구조체 정의 ---
class SensorData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp",     ctypes.c_uint32),
        ("sensor_update", ctypes.c_uint8),
        ("ax",            ctypes.c_float),
        ("ay",            ctypes.c_float),
        ("az",            ctypes.c_float),
        ("gx",            ctypes.c_float),
        ("gy",            ctypes.c_float),
        ("gz",            ctypes.c_float),
        ("w",             ctypes.c_float),
        ("x",             ctypes.c_float),
        ("y",             ctypes.c_float),
        ("z",             ctypes.c_float),
        ("raw_p",         ctypes.c_float),
        ("filt_p",        ctypes.c_float),
        ("filt_alt",      ctypes.c_float),
        ("alt_baro",      ctypes.c_float),
        ("alt_imu",       ctypes.c_float),
        ("filt_velocity", ctypes.c_float),
        ("vel_z_imu",     ctypes.c_float),
        ("vel_z_baro",    ctypes.c_float),
        ("flight_state",  ctypes.c_uint8),
        ("ej1_state",     ctypes.c_uint8),
        ("ej2_state",     ctypes.c_uint8),
        ("sep_state",     ctypes.c_uint8),
    ]

# --- [2] 변환 로직 함수 (시간 변환 제거됨) ---
def convert_bin_to_excel(input_path, output_path, status_callback):
    try:
        struct_size = ctypes.sizeof(SensorData)
        data_list = []
        
        status_callback(f"파일 읽는 중... ({input_path})")
        
        with open(input_path, "rb") as f:
            while True:
                chunk = f.read(struct_size)
                if not chunk or len(chunk) < struct_size:
                    break
                
                record = SensorData.from_buffer_copy(chunk)
                
                # 구조체 -> 딕셔너리 변환
                row = {}
                for field_name, field_type in record._fields_:
                    row[field_name] = getattr(record, field_name)
                data_list.append(row)

        if not data_list:
            status_callback("오류: 데이터가 비어있습니다.")
            return False

        status_callback(f"데이터 {len(data_list)}개 변환 중...")
        
        # DataFrame 생성
        df = pd.DataFrame(data_list)

        # [수정됨] 시간 단위 변환 로직 삭제함
        # 이제 timestamp는 아두이노에서 보낸 micros() 값 그대로 저장됩니다.

        # 엑셀 저장
        status_callback("엑셀 파일 저장 중...")
        df.to_excel(output_path, index=False, engine='openpyxl')
        
        status_callback(f"완료! 저장됨: {output_path}")
        return True

    except Exception as e:
        status_callback(f"에러 발생: {str(e)}")
        return False

# --- [3] GUI 클래스 ---
class RocketLogConverterApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Rocket Log Converter (.bin -> .xlsx)")
        self.root.geometry("400x300")
        self.root.resizable(False, False)

        # UI 변수
        self.input_file_path = tk.StringVar()
        self.output_file_path = tk.StringVar()

        # 1. 입력 파일 선택 영역
        lbl_input = tk.Label(root, text="입력 파일 (.bin):")
        lbl_input.grid(row=0, column=0, sticky="w", padx=10, pady=(20, 5))
        
        entry_input = tk.Entry(root, textvariable=self.input_file_path, width=40, state='readonly')
        entry_input.grid(row=1, column=0, padx=10, pady=5)
        
        btn_input = tk.Button(root, text="파일 선택", command=self.select_input_file)
        btn_input.grid(row=1, column=1, padx=5, pady=5)

        # 2. 출력 파일 선택 영역
        lbl_output = tk.Label(root, text="저장할 경로 및 이름 (.xlsx):")
        lbl_output.grid(row=2, column=0, sticky="w", padx=10, pady=(10, 5))
        
        entry_output = tk.Entry(root, textvariable=self.output_file_path, width=40)
        entry_output.grid(row=3, column=0, padx=10, pady=5)
        
        btn_output = tk.Button(root, text="경로 변경", command=self.select_output_file)
        btn_output.grid(row=3, column=1, padx=5, pady=5)

        # 3. 변환 버튼
        self.btn_convert = tk.Button(root, text="엑셀로 변환하기", command=self.run_conversion, 
                                     bg="#4CAF50", fg="white", font=("Arial", 10, "bold"), height=2)
        self.btn_convert.grid(row=4, column=0, columnspan=2, sticky="ew", padx=10, pady=20)

        # 4. 상태 표시줄
        self.lbl_status = tk.Label(root, text="준비됨", fg="gray")
        self.lbl_status.grid(row=5, column=0, columnspan=2, pady=5)

    def select_input_file(self):
        filename = filedialog.askopenfilename(
            title="바이너리 로그 파일 선택",
            filetypes=[("Binary Files", "*.bin"), ("All Files", "*.*")]
        )
        if filename:
            self.input_file_path.set(filename)
            # 입력 파일을 선택하면 자동으로 출력 파일명 추천
            suggested_output = filename.replace(".bin", ".xlsx")
            self.output_file_path.set(suggested_output)
            self.lbl_status.config(text="파일 선택됨. 저장 경로를 확인하세요.", fg="black")

    def select_output_file(self):
        # 입력 파일명을 기반으로 초기값 설정
        initial_file = os.path.basename(self.input_file_path.get()).replace(".bin", ".xlsx")
        
        filename = filedialog.asksaveasfilename(
            title="저장할 경로 선택",
            defaultextension=".xlsx",
            initialfile=initial_file,
            filetypes=[("Excel Files", "*.xlsx")]
        )
        if filename:
            self.output_file_path.set(filename)

    def update_status(self, message):
        self.lbl_status.config(text=message)
        self.root.update_idletasks() # UI 즉시 갱신

    def run_conversion(self):
        in_path = self.input_file_path.get()
        out_path = self.output_file_path.get()

        if not in_path:
            messagebox.showwarning("경고", "입력 파일을 먼저 선택해주세요.")
            return

        if not out_path:
            messagebox.showwarning("경고", "저장할 경로를 설정해주세요.")
            return

        # 버튼 비활성화 (중복 클릭 방지)
        self.btn_convert.config(state="disabled", text="변환 중...")
        
        # 변환 실행
        success = convert_bin_to_excel(in_path, out_path, self.update_status)
        
        # 버튼 복구
        self.btn_convert.config(state="normal", text="엑셀로 변환하기")

        if success:
            messagebox.showinfo("성공", f"변환이 완료되었습니다!\n저장 경로: {out_path}")
            self.lbl_status.config(text="변환 완료", fg="blue")
        else:
            messagebox.showerror("실패", "변환 중 오류가 발생했습니다. 상태 메시지를 확인하세요.")
            self.lbl_status.config(fg="red")

# --- [4] 메인 실행 ---
if __name__ == "__main__":
    root = tk.Tk()
    app = RocketLogConverterApp(root)
    root.mainloop()