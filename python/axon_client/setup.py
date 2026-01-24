# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""Setup configuration for axon_client package."""

from pathlib import Path
from setuptools import setup, find_packages

this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text(encoding="utf-8")

setup(
    name="axon-client",
    version="0.1.0",
    author="ArcheBase",
    author_email="info@archebase.com",
    description="Python HTTP client for Axon recorder",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/archebase/axon",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "License :: OSI Approved :: MulanPSL License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
    ],
    python_requires=">=3.8",
    install_requires=[
        "requests>=2.31.0",
        "aiohttp>=3.9.0",
    ],
    extras_require={
        "zenoh": ["eclipse-zenoh>=1.0.0"],
        "dev": [
            "pytest>=7.4.0",
            "pytest-asyncio>=0.21.0",
            "pytest-cov>=4.1.0",
            "responses>=0.23.0",
            "aioresponses>=0.7.0",
            "black>=23.0.0",
            "mypy>=1.5.0",
            "flake8>=6.1.0",
        ],
    },
)
