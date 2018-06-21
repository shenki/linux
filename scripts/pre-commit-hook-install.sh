#!/bin/sh

REPO_ROOT=$(git rev-parse --show-toplevel)


if [ -d "${REPO_ROOT}/.git" ]; then
        GIT_DIR=${REPO_ROOT}/.git
elif [ -f "${REPO_ROOT}/.git" ]; then
        GIT_SUB_PATH=$(cat ${REPO_ROOT}/.git|cut -d " " -f 2)
        GIT_DIR=${REPO_ROOT}/${GIT_SUB_PATH}
else
        echo ".git not found"
        exit -1
fi

if [ -e "${GIT_DIR}/hooks/pre-commit" ]; then
        mv ${GIT_DIR}/hooks/pre-commit ${GIT_DIR}/hooks/pre-commit.backup
fi

cp ${REPO_ROOT}/scripts/git-pre-commit-hook ${GIT_DIR}/hooks/pre-commit
chmod 777 ${GIT_DIR}/hooks/pre-commit
echo "install success"
