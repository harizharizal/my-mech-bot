FROM python:3.13-slim
WORKDIR /bot
COPY mech_survivor.py /bot/mech_survivor.py
ENTRYPOINT ["python3", "-u", "/bot/mech_survivor.py"]
