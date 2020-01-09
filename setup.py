import setuptools

with open("readme.md", "r") as fh:
    long_description = fh.read()

    setuptools.setup(
                name="pyQpController", # Replace with your own username
                version="0.0.1",
                author="Yuquan Wang",
                author_email="wyqsnddd@gmail.com",
                description="Python implementation of a multi-objective controller using quadratic programming.",
                long_description=long_description,
                long_description_content_type="text/markdown",
                url="https://github.com/wyqsnddd/pyQpController",
                packages=setuptools.find_packages(),
                classifiers=[
                    "Programming Language :: Python :: 3",
                    "License :: OSI Approved :: LGPL",
                    "Operating System :: OS Independent",
                    ],
                python_requires='>=3.5',
                )

