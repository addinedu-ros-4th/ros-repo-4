#pycryptodome
from Crypto.Cipher import AES
from Crypto.Random import get_random_bytes

# 키와 데이터
key = get_random_bytes(16)
data = b"0000"

# 암호화
cipher = AES.new(key, AES.MODE_EAX)
ciphertext, tag = cipher.encrypt_and_digest(data)

print("Cipher text:", ciphertext)

# 복호화
cipher = AES.new(key, AES.MODE_EAX, nonce=cipher.nonce)
decrypted_data = cipher.decrypt_and_verify(ciphertext, tag)

print("Decrypted text:", decrypted_data.decode('utf-8'))
