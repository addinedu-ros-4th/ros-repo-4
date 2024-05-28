from Crypto.Cipher import AES
from Crypto.Random import get_random_bytes

# AES 키 생성 (16 바이트)
key = get_random_bytes(16)

# 암호화할 데이터
data = b"0000"

# 암호화
cipher = AES.new(key, AES.MODE_EAX)
ciphertext, tag = cipher.encrypt_and_digest(data)

# 암호화 결과 출력
print("Ciphertext:", ciphertext)
print("Tag:", tag)
print("Nonce:", cipher.nonce)

# 복호화
cipher = AES.new(key, AES.MODE_EAX, nonce=cipher.nonce)
decrypted_data = cipher.decrypt_and_verify(ciphertext, tag)

# 복호화 결과 출력
print("Decrypted text:", decrypted_data.decode('utf-8'))