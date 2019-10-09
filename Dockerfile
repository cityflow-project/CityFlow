FROM ubuntu:16.04

# c++ dependencies
RUN apt update && \
    apt-get install -y build-essential cmake wget git

# install Miniconda Python 3.6
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV PATH /opt/conda/bin:$PATH

RUN wget -P /tmp/ https://repo.continuum.io/miniconda/Miniconda3-4.5.4-Linux-x86_64.sh && \
    /bin/bash /tmp/Miniconda3-4.5.4-Linux-x86_64.sh -b -p /opt/conda && \
    rm /tmp/Miniconda3-4.5.4-Linux-x86_64.sh && \
    ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
    echo ". /opt/conda/etc/profile.d/conda.sh" >> ~/.bashrc

# install cityflow
COPY . /home/cityflow
RUN pip install flask && \
    cd /home/cityflow && \
    pip install .