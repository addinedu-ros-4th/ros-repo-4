from Crypto.Cipher import AES
from Crypto.Util.Padding import pad, unpad
import hashlib

print("**암호화**")

# 사용자 암호를 입력받는다.
# pycryptodome은 오직 bytes형만 처리할 수 있다. encode()를 이용 사용자 암호를 bytes형으로 변환한다.
original_password = input("password: ").encode('utf8')

# AES암호화를 위해서는 32바이트 key가 필요하다. 
# hashlib을 이용하여 사용자 암호를 32바이트로 변환한 후,이를 key로 사용한다.
key = hashlib.pbkdf2_hmac(hash_name='sha256', password=original_password, salt=b'$3kj##agh_', iterations=100000)


#암호화할 비밀도 encode("utf8")을 이용 bytes로 변환한다.
text = input("암호화 하고자 하는 비밀: ").encode("utf8")

#암호화를 실행하기 위해서, 다음과 같이 AES 개체를 만든다.
aes = AES.new(key, AES.MODE_ECB)

#AES를 이용 암호화하려면, 암호화의 대상인 text가 16, 32, 64, 128, 256 바이트의 블록들이어야 한다.
#위와 같이 암호화 대상인 text를 16, 32, 64, 128, 256 바이트의 블록들로 만드는 것을 padding이라고 한다.
#내장된 pad 모듈을 이용하여 암호화를 하려는 text를 블록들로 변경한다.
Block_Size = 256
padded_text = pad(text, Block_Size)

#암호화를 실행하고, 암호화된 결과물을 확인한다.
encrypted_text = aes.encrypt(padded_text)
print("암호화된 비밀: ", encrypted_text)

