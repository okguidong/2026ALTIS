import struct
import csv
import os

def bin_to_csv(bin_file, csv_file):
    # 1. 구조체 포맷 정의
    # < : 리틀 엔디안
    # I : timestamp (uint32_t)
    # f : float (총 14개: acc 3 + gyro 3 + quat 4 + press 2 + alt 1 + vel 1)
    # B : uint8_t (총 4개: state들)
    struct_fmt = '<IffffffffffffffBBBB'
    
    # 구조체 크기 계산 (64 bytes 예상)
    packet_size = struct.calcsize(struct_fmt)
    
    # 2. CSV 헤더 정의 (순서대로)
    headers = [
        'timestamp(us)', 
        'ax', 'ay', 'az', 
        'gx', 'gy', 'gz', 
        'qw', 'qx', 'qy', 'qz', 
        'raw_pressure', 'filt_pressure', 'filt_altitude', 'velocity', 
        'flight_state', 'ej1_state', 'ej2_state', 'sep_state'
    ]

    print(f"--- 변환 설정 정보 ---")
    print(f"입력 파일: {bin_file}")
    print(f"출력 파일: {csv_file}")
    print(f"패킷 크기: {packet_size} bytes")
    
    data_count = 0

    try:
        if not os.path.exists(bin_file):
            print(f"오류: '{bin_file}' 파일을 찾을 수 없습니다.")
            return

        with open(bin_file, 'rb') as f_in, open(csv_file, 'w', newline='') as f_out:
            writer = csv.writer(f_out)
            writer.writerow(headers) # 헤더 쓰기

            while True:
                # 64바이트씩 읽기
                chunk = f_in.read(packet_size)
                
                # 데이터가 없으면 종료
                if not chunk:
                    break
                
                # 데이터가 64바이트보다 작으면 (파일 끝부분 깨짐 등) 무시
                if len(chunk) < packet_size:
                    print(f"경고: 마지막에 완전하지 않은 데이터({len(chunk)} bytes)가 있어 무시합니다.")
                    break
                
                # 바이너리 -> 데이터 풀기
                data = struct.unpack(struct_fmt, chunk)
                
                # CSV에 쓰기
                writer.writerow(data)
                data_count += 1

        print(f"--- 변환 완료 ---")
        print(f"총 {data_count}개의 데이터 패킷이 저장되었습니다.")

    except Exception as e:
        print(f"에러 발생: {e}")

if __name__ == "__main__":
    # 여기에 변환할 bin 파일 경로를 입력하세요
    input_filename = r"GUI\basic\flight_log_002.bin"   # 예: 'flight_log.bin'
    output_filename = r"GUI\basic\data.csv"  # 저장될 csv 파일 이름
    
    bin_to_csv(input_filename, output_filename) 