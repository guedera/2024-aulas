import styles from './botao-adicionar.module.css';
import { LuPlusCircle } from "react-icons/lu";

export default function BotaoAdicionar(props) {
    return (
        <button className={styles.botaoAdicionar} onClick={props.onClick}>
            <LuPlusCircle className={styles.icone} />
        </button>
    );
}
