import subprocess
import datetime

def capture_webcam_image(output_file="captured_image.jpg", device="/dev/video0"):
    """
    FFmpeg를 사용하여 웹캠 이미지를 캡처합니다.

    :param output_file: 저장할 이미지 파일 이름
    :param device: 웹캠 디바이스 (기본값: /dev/video0)
    """
    try:
        # FFmpeg 명령어 구성
        command = [
            "ffmpeg",
            "-y",  # 기존 파일 덮어쓰기
            "-f", "v4l2",  # Video4Linux2 포맷
            "-i", device,  # 입력 디바이스
            "-vframes", "1",  # 1 프레임만 캡처
            output_file  # 출력 파일
        ]

        # FFmpeg 실행
        subprocess.run(command, check=True)

        print(f"Image captured and saved as {output_file}")
    except subprocess.CalledProcessError as e:
        print(f"Error capturing image: {e}")
    except FileNotFoundError:
        print("FFmpeg is not installed or not found in PATH. Please install it.")

# 캡처 실행
if __name__ == "__main__":
    filename = f"question.png"
    capture_webcam_image(output_file=filename, device="/dev/video0")
