#!/usr/bin/env zsh

set -exuo pipefail

sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
git clone https://github.com/zsh-users/zsh-completions.git ${ZSH_CUSTOM:-${ZSH:-~/.oh-my-zsh}/custom}/plugins/zsh-completions
git clone https://github.com/zsh-users/zsh-autosuggestions.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k
sed -i "s/^plugins=\(.*\)/plugins=\(debian gpg-agent cp git zsh-syntax-highlighting zsh-autosuggestions\)/" ${HOME}/.zshrc
sed -i "/^plugins=\(.*\)/afpath+=\$\{ZSH_CUSTOM:-\$\{ZSH:-\~\/\.oh-my-zsh\}/custom\}/plugins\/zsh-completions\/src\/\nautoload -U compinit && compinit" ${HOME}/.zshrc
sed -i "s/^ZSH_THEME=\".*\"/ZSH_THEME=\"powerlevel10k\/powerlevel10k\"/" ${HOME}/.zshrc
sed -i "$ a\source \~\/.oh-my-zsh\/custom\/themes\/powerlevel10k\/config\/p10k-robbyrussell.zsh" ${HOME}/.zshrc
