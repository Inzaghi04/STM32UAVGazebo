PYTHON := python3
MAIN := forward_data.py

.PHONY: run install clean

# Cài đặt thư viện từ requirements.txt
install:
	$(PYTHON) -m pip install -r requirements.txt

# Chạy chương trình chính
run:
	$(PYTHON) $(MAIN)

# Dọn dẹp file rác
clean:
	rm -rf __pycache__ *.pyc
